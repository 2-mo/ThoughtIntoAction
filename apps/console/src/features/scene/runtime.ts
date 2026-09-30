// @ts-nocheck
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { TargetPoseEditor, TARGET_EDIT_STATE } from "./target-pose-editor";
import baseLinkMeshUrl from "@weights/geometry/x5/base_link.STL?url";
import d405MeshUrl from "@weights/geometry/intel-realsense-d405.stl?url";
import link1MeshUrl from "@weights/geometry/x5/link1.STL?url";
import link2MeshUrl from "@weights/geometry/x5/link2.STL?url";
import link3MeshUrl from "@weights/geometry/x5/link3.STL?url";
import link4MeshUrl from "@weights/geometry/x5/link4.STL?url";
import link5MeshUrl from "@weights/geometry/x5/link5.STL?url";
import link6MeshUrl from "@weights/geometry/x5/link6.STL?url";
import {
  LOCAL_APPROACH_AXIS,
  cloneTargetPose,
  quaternionFromArray,
  quaternionFromRpy,
  quaternionToArray,
} from "./target-pose-math";

import {
  cancelParkPreview,
  getMotionSpeedProgress,
  getMotionSpeedParams,
  getMotionSpeedPercent,
  getArmState,
  requestParkAction,
  requestEmergencyStop,
  sendCommand,
  sendGripperServoMessage,
  sendMotionCommand,
  sendServoMessage,
  setControlMessage,
} from "../control/runtime";

const MODEL_REVISION = "8ccbf0aa8d4288f2721c41a735e5ac4c389bb13b";
const MODEL_URLS = [
  baseLinkMeshUrl,
  link1MeshUrl,
  link2MeshUrl,
  link3MeshUrl,
  link4MeshUrl,
  link5MeshUrl,
  link6MeshUrl,
];
const MODEL_CACHE_NAME = `arx5-x5-stl-${MODEL_REVISION}`;
const MODEL_FETCH_TIMEOUT_MS = 8_000;
const D405_MODEL_URL = d405MeshUrl;
const TABLE_CALIBRATION_STORAGE_KEY = "arx5-table-calibration-v7";
const LEGACY_TABLE_CALIBRATION_STORAGE_KEYS = [
  "arx5-table-calibration-v6",
  "arx5-table-calibration-v5",
  "arx5-table-calibration-v4",
];
const CAMERA_CALIBRATION_STORAGE_KEY = "arx5-camera-calibration-v3";
const LEGACY_CAMERA_CALIBRATION_STORAGE_KEY = "arx5-camera-calibration-v2";
const LIGHT_CALIBRATION_STORAGE_KEY = "arx5-light-calibration-v1";
const D455_RGB_VERTICAL_FOV_DEG = 65;
const DEFAULT_TABLE_CALIBRATION = Object.freeze({
  widthMm: 2000,
  depthMm: 1000,
  thicknessMm: 35,
  // The X5 base mesh ends at X = -30 mm.  With the table's 1000 mm short
  // side along X (Yaw 90°), a center at +470 mm puts its long rear edge at
  // exactly that base outer edge and leaves the arm facing along the short
  // side into the tabletop.
  xMm: 470,
  yMm: -671,
  zMm: 0,
  yawDeg: 90,
});
const LEGACY_DEFAULT_TABLE_CALIBRATION = Object.freeze({
  widthMm: 2000,
  depthMm: 1000,
  thicknessMm: 35,
  xMm: 450,
  yMm: -700,
  zMm: 0,
  yawDeg: 90,
});
const LEGACY_NUDGED_DEFAULT_TABLE_CALIBRATION = Object.freeze({
  ...LEGACY_DEFAULT_TABLE_CALIBRATION,
  xMm: 462,
  yMm: -671,
});
const DEFAULT_CAMERA_CALIBRATION = Object.freeze({
  xMm: 900,
  yMm: 0,
  zMm: 205,
  rollDeg: 0,
  pitchDeg: 0,
  yawDeg: 90,
});
const DEFAULT_LIGHT_CALIBRATION = Object.freeze({
  xMm: 450,
  yMm: -550,
  zMm: 1200,
  intensity: 3.4,
});
const JOINT_NAMES = ["底座旋转", "肩部", "肘部", "腕部 1", "腕部 2", "末端旋转"];
const LIMIT_MARGIN_DEG = 1;
const EEF_OFFSET_LINK6_M = 0.145;
// Intel realsense2_description 4.55.1, _d405.urdf.xacro. The STL itself is
// authored in millimetres and its origin is on the front plate between the IR
// camera axes.
const D405_STL_SCALE_M = 0.001;
const D405_LINK_TO_VISUAL_XYZ_M = [0.0038, -0.009, 0];
const D405_LINK_TO_VISUAL_RPY_RAD = [Math.PI / 2, 0, Math.PI / 2];
const D405_LINK_TO_OPTICAL_RPY_RAD = [-Math.PI / 2, 0, -Math.PI / 2];
const D405_FALLBACK_BODY_XYZ_M = [-0.0078, -0.009, 0];
const D405_BODY_SIZE_M = [0.023, 0.042, 0.042];
const CARTESIAN_DRAG_SNAP_MM = 0.5;
const LIVE_PREVIEW_INTERVAL_MS = 32;
const SERVO_HEARTBEAT_INTERVAL_MS = 50;
const GRIPPER_KEY_HEARTBEAT_INTERVAL_MS = 80;
const GRIPPER_FINGER_THICKNESS_M = 0.013;
const DISCONNECTED_GRIPPER_WIDTH_MM = 0;
const CONTROL_MODES = ["xyz", "joint", "hand"];
const CARTESIAN_PLANES_BY_CODE = {
  KeyZ: {
    key: "z",
    name: "XY",
    axes: ["x", "y"],
    normal: new THREE.Vector3(0, 0, 1),
    color: 0x74a9ff,
  },
  KeyX: {
    key: "x",
    name: "XZ",
    axes: ["x", "z"],
    normal: new THREE.Vector3(0, 1, 0),
    color: 0x70e89f,
  },
  KeyC: {
    key: "c",
    name: "YZ",
    axes: ["y", "z"],
    normal: new THREE.Vector3(1, 0, 0),
    color: 0xff7777,
  },
};
const DISCONNECTED_DISPLAY_ANGLES = [0, 0, 0, 0, 0, 0];
const SERVO_PREVIEW_COLOR = 0x35d6ff;
const LIMIT_PREVIEW_COLOR = 0xff665f;
const PENDING_PATH_COLOR = 0xffa43a;
const EXECUTED_PATH_COLOR = 0x38a7ff;
const MAX_EXECUTED_PATH_POINTS = 360;
const PATH_GEOMETRY_UPDATE_INTERVAL_MS = 80;
const PATH_FLOW_UPDATE_INTERVAL_MS = 32;
const ARM_STATE_UI_INTERVAL_MS = 100;
const MOTION_EXECUTION_ACTIONS = Object.freeze([
  "move_xyz",
  "move_joint",
  "move_joint_target",
  "reset_joint",
  "park",
  "cube_pick_execute",
  "cube_move_execute",
  "cube_place_execute",
  "cube_stack_execute",
]);
const J3_LINK_MATERIAL_INDEX = 3;
const J3_METAL_COLOR = 0xc8d0d5;
const PLASTIC_BLACK_COLOR = 0x171a1d;
const gripperGeometryClassifications = new WeakMap();
const gripperGeometryDeformations = new WeakMap();

// Joint origins and axes come directly from the upstream X5.urdf.  Each
// motion group is kept separate from its fixed origin transform so that the
// green selection ring stays at the physical joint pivot.
const JOINT_SPECS = [
  { origin: [0, 0, 0.0603], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { origin: [0.02, 0, 0.0402], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { origin: [-0.264, 0, 0], rpy: [-Math.PI, 0, 0], axis: [0, 1, 0] },
  { origin: [0.245, 0, -0.056], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { origin: [0.06575, -0.001, -0.0825], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { origin: [0.02845, 0, 0.0825], rpy: [-Math.PI, 0, 0], axis: [1, 0, 0] },
];

// Bounding boxes measured from the official meshes.  They provide a usable
// low-poly fallback when a packaged mesh cannot be loaded.
const FALLBACK_LINKS = [
  { size: [0.060, 0.060, 0.0625], center: [0, 0, 0.03125] },
  { size: [0.0593, 0.0785, 0.0575], center: [0.0078, 0, 0.0289] },
  { size: [0.320, 0.0665, 0.056], center: [-0.132, 0.003, 0] },
  { size: [0.291, 0.0765, 0.102], center: [0.128, 0, -0.0333] },
  { size: [0.113, 0.0607, 0.102], center: [0.0392, -0.0014, -0.0335] },
  { size: [0.060, 0.060, 0.111], center: [0, 0, 0.0553] },
  { size: [0.147, 0.182, 0.061], center: [0.0733, 0, 0] },
];

const elements = {
  canvas: document.querySelector("#robotCanvas"),
  loading: document.querySelector("#robotModelLoading"),
  loadingText: document.querySelector("#robotModelLoadingText"),
  modelStatus: document.querySelector("#robotModelStatus"),
  resetViewButton: document.querySelector("#robotResetViewButton"),
  resetViewButtonLabel: document.querySelector("#robotResetViewButton span"),
  tableCalibrationButton: document.querySelector("#tableCalibrationButton"),
  tableCalibrationPanel: document.querySelector("#tableCalibrationPanel"),
  tableCalibrationClose: document.querySelector("#tableCalibrationClose"),
  tableCalibrationReset: document.querySelector("#tableCalibrationReset"),
  tableFocusButton: document.querySelector("#tableFocusButton"),
  tableEdgeEditButton: document.querySelector("#tableEdgeEditButton"),
  tableAutoAlignButton: document.querySelector("#tableAutoAlignButton"),
  tableAutoAlignStatus: document.querySelector("#tableAutoAlignStatus"),
  tableInputs: {
    widthMm: document.querySelector("#tableWidthMm"),
    depthMm: document.querySelector("#tableDepthMm"),
    thicknessMm: document.querySelector("#tableThicknessMm"),
    xMm: document.querySelector("#tablePositionXmm"),
    yMm: document.querySelector("#tablePositionYmm"),
    zMm: document.querySelector("#tablePositionZmm"),
    yawDeg: document.querySelector("#tableYawDeg"),
  },
  cameraCalibrationButton: document.querySelector("#cameraCalibrationButton"),
  cameraCalibrationPanel: document.querySelector("#cameraCalibrationPanel"),
  cameraCalibrationClose: document.querySelector("#cameraCalibrationClose"),
  cameraCalibrationReset: document.querySelector("#cameraCalibrationReset"),
  cameraFocusButton: document.querySelector("#cameraFocusButton"),
  cameraViewButton: document.querySelector("#cameraViewButton"),
  cameraInputs: {
    xMm: document.querySelector("#cameraPositionXmm"),
    yMm: document.querySelector("#cameraPositionYmm"),
    zMm: document.querySelector("#cameraPositionZmm"),
    rollDeg: document.querySelector("#cameraRollDeg"),
    pitchDeg: document.querySelector("#cameraPitchDeg"),
    yawDeg: document.querySelector("#cameraYawDeg"),
  },
  lightCalibrationButton: document.querySelector("#lightCalibrationButton"),
  lightCalibrationPanel: document.querySelector("#lightCalibrationPanel"),
  lightCalibrationClose: document.querySelector("#lightCalibrationClose"),
  lightCalibrationReset: document.querySelector("#lightCalibrationReset"),
  lightInputs: {
    xMm: document.querySelector("#lightPositionXmm"),
    yMm: document.querySelector("#lightPositionYmm"),
    zMm: document.querySelector("#lightPositionZmm"),
    intensity: document.querySelector("#lightIntensity"),
  },
  targetEditBadge: document.querySelector("#targetEditBadge"),
  syncPreviewButton: document.querySelector("#robotSyncPreviewButton"),
  controlModeButtons: [...document.querySelectorAll("#robotControlModes button")],
  controlModes: document.querySelector("#robotControlModes"),
  advancedJoints: document.querySelector(".joint-control-pane > .advanced-joints"),
  speedControl: document.querySelector("#motionSpeedControl"),
  speedInput: document.querySelector("#motionSpeed"),
  speedValue: document.querySelector("#motionSpeedValue"),
  xyzPane: document.querySelector("#xyzControlPane"),
  handPane: document.querySelector("#handControlPane"),
  handState: document.querySelector("#robotHandState"),
  servoEnabled: document.querySelector("#cartesianServoEnabled"),
  pathModeButtons: [...document.querySelectorAll("#xyzPathModes button")],
  pathModes: document.querySelector("#xyzPathModes"),
  jointPane: document.querySelector("#jointControlPane"),
  xyzCurrent: document.querySelector("#xyzCurrent"),
  xyzInputs: {
    x: document.querySelector("#xyzDeltaX"),
    y: document.querySelector("#xyzDeltaY"),
    z: document.querySelector("#xyzDeltaZ"),
  },
  xyzQuickButtons: [...document.querySelectorAll("#xyzQuickMoves button")],
  xyzClearButton: document.querySelector("#xyzClearButton"),
  xyzPreviewButton: document.querySelector("#xyzPreviewButton"),
  xyzExecuteButton: document.querySelector("#xyzExecuteButton"),
  xyzStatus: document.querySelector("#xyzStatus"),
  targetEditingState: document.querySelector("#targetEditingState"),
  targetApproach: {
    x: document.querySelector("#targetApproachX"),
    y: document.querySelector("#targetApproachY"),
    z: document.querySelector("#targetApproachZ"),
  },
  targetRoll: document.querySelector("#targetRollDeg"),
  targetQuaternion: document.querySelector("#targetQuaternion"),
  gripperWidth: document.querySelector("#gripperWidth"),
  gripperHint: document.querySelector("#gripperHint"),
  gripperTorque: document.querySelector("#gripperTorque"),
  gripperMinimum: document.querySelector("#gripperMinimum"),
  jointButtons: [...document.querySelectorAll("#robotJointButtons button")],
  actual: document.querySelector("#robotJointActual"),
  target: document.querySelector("#robotJointTarget"),
  delta: document.querySelector("#robotJointDelta"),
  dial: document.querySelector("#jointDial"),
  dialNeedle: document.querySelector("#jointDialNeedle"),
  dialLabel: document.querySelector("#jointDialLabel"),
  nudgeButtons: [...document.querySelectorAll("#robotJointNudges button")],
  targetInput: document.querySelector("#robotJointTargetInput"),
  limit: document.querySelector("#robotJointLimit"),
  previewResetButton: document.querySelector("#robotPreviewResetButton"),
  executeButton: document.querySelector("#robotExecuteButton"),
  previewStatus: document.querySelector("#robotPreviewStatus"),
};

// Explicitly override browser form-state restoration so every session starts
// from the documented 10% speed.
elements.speedInput.value = "5";

const renderer = new THREE.WebGLRenderer({
  canvas: elements.canvas,
  antialias: true,
  alpha: true,
  powerPreference: "high-performance",
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
const handeyePlanArrows = new THREE.Group();
handeyePlanArrows.name = "handeye-calibration-waypoints";
handeyePlanArrows.visible = false;
scene.add(handeyePlanArrows);

// Before calibration the D405 mount is only a visual TCP reference. The
// target pose itself is measured in D405 optical coordinates and never feeds
// robot motion or the solver from this visual overlay.
const handeyeBoardPreview = new THREE.Group();
handeyeBoardPreview.name = "handeye-d405-observed-target";
const handeyeBoardFace = new THREE.Mesh(
  new THREE.PlaneGeometry(0.10, 0.10),
  new THREE.MeshBasicMaterial({
    color: 0xffd85a,
    transparent: true,
    opacity: 0.42,
    side: THREE.DoubleSide,
    depthTest: false,
  }),
);
const handeyeBoardOutline = new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.PlaneGeometry(0.10, 0.10)),
  new THREE.LineBasicMaterial({ color: 0xffe7a3, depthTest: false }),
);
const handeyeBoardAxes = new THREE.AxesHelper(0.065);
handeyeBoardPreview.add(handeyeBoardFace, handeyeBoardOutline, handeyeBoardAxes);
handeyeBoardPreview.visible = false;
handeyeBoardPreview.renderOrder = 20;
scene.add(handeyeBoardPreview);
let handeyeBoardCameraTarget = null;
const OPENCV_CAMERA_TO_VIEWER = new THREE.Matrix4().makeRotationX(Math.PI);
scene.fog = new THREE.FogExp2(0x0d0d0d, 0.3);
scene.environmentIntensity = 0.72;
let sceneEnvironmentTarget = null;

function rebuildSceneEnvironment() {
  const previousTarget = sceneEnvironmentTarget;
  const environmentGenerator = new THREE.PMREMGenerator(renderer);
  sceneEnvironmentTarget = environmentGenerator.fromScene(new RoomEnvironment(), 0.04);
  scene.environment = sceneEnvironmentTarget.texture;
  environmentGenerator.dispose();
  previousTarget?.dispose();
}
rebuildSceneEnvironment();

const DEFAULT_SCENE_CAMERA_FOV_DEG = 39;
const camera = new THREE.PerspectiveCamera(DEFAULT_SCENE_CAMERA_FOV_DEG, 1, 0.01, 8);
camera.up.set(0, 0, 1);
const DEFAULT_CAMERA_POSITION = new THREE.Vector3(-0.72, -0.72, 0.8);
const DEFAULT_CAMERA_TARGET = new THREE.Vector3(0.22, -0.2, 0.22);
camera.position.copy(DEFAULT_CAMERA_POSITION);

const controls = new OrbitControls(camera, elements.canvas);
controls.target.copy(DEFAULT_CAMERA_TARGET);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.28;
controls.maxDistance = 2.2;
controls.maxPolarAngle = Math.PI * 0.93;
controls.update();

scene.add(new THREE.HemisphereLight(0xffffff, 0x171717, 2.9));
const keyLight = new THREE.DirectionalLight(0xffffff, 3.4);
keyLight.position.set(0.45, -0.55, 1.2);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.camera.near = 0.05;
keyLight.shadow.camera.far = 3.2;
keyLight.shadow.camera.left = -2.1;
keyLight.shadow.camera.right = 2.1;
keyLight.shadow.camera.top = 2.1;
keyLight.shadow.camera.bottom = -1.6;
keyLight.shadow.bias = -0.00025;
keyLight.shadow.normalBias = 0.018;
scene.add(keyLight);
const keyLightHelper = new THREE.DirectionalLightHelper(keyLight, 0.16, 0xffe6ad);
keyLightHelper.visible = false;
scene.add(keyLightHelper);
let lightCalibrationState = loadLightCalibration();
let lightCalibrationEditing = false;
applyLightCalibration(lightCalibrationState, { persist: false, syncInputs: true });
const fillLight = new THREE.DirectionalLight(0xa9b8b2, 1.05);
fillLight.position.set(-0.75, 0.45, 0.45);
scene.add(fillLight);

const grid = new THREE.GridHelper(1.4, 28, 0x606060, 0x2c2c2c);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.002;
grid.material.transparent = true;
grid.material.opacity = 0.66;
scene.add(grid);

const baseAxes = new THREE.AxesHelper(0.115);
baseAxes.position.z = 0.003;
scene.add(baseAxes);

const teleoperationWorkspaceBox = new THREE.LineSegments(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({
    color: 0x79e0bf,
    transparent: true,
    opacity: 0.48,
    depthTest: false,
  }),
);
teleoperationWorkspaceBox.name = "teleoperation-workspace-box";
teleoperationWorkspaceBox.renderOrder = 18;
teleoperationWorkspaceBox.visible = false;
teleoperationWorkspaceBox.userData.configured = false;
scene.add(teleoperationWorkspaceBox);
let teleoperationWorkspaceSignature = null;

function updateTeleoperationWorkspaceBox(config) {
  const minimum = config?.workspace?.minimum_m;
  const maximum = config?.workspace?.maximum_m;
  const valid = [minimum, maximum].every((vector) => (
    Array.isArray(vector)
    && vector.length === 3
    && vector.every((value) => Number.isFinite(Number(value)))
  ));
  if (!valid) {
    teleoperationWorkspaceBox.userData.configured = false;
    teleoperationWorkspaceBox.visible = false;
    teleoperationWorkspaceSignature = null;
    return;
  }
  const bounds = [...minimum, ...maximum].map(Number);
  const minimumValues = bounds.slice(0, 3);
  const maximumValues = bounds.slice(3);
  const signature = bounds.join(",");
  if (signature !== teleoperationWorkspaceSignature) {
    const size = new THREE.Vector3(
      maximumValues[0] - minimumValues[0],
      maximumValues[1] - minimumValues[1],
      maximumValues[2] - minimumValues[2],
    );
    const center = new THREE.Vector3(
      (minimumValues[0] + maximumValues[0]) / 2,
      (minimumValues[1] + maximumValues[1]) / 2,
      (minimumValues[2] + maximumValues[2]) / 2,
    );
    const solidGeometry = new THREE.BoxGeometry(size.x, size.y, size.z);
    const edgeGeometry = new THREE.EdgesGeometry(solidGeometry);
    solidGeometry.dispose();
    teleoperationWorkspaceBox.geometry.dispose();
    teleoperationWorkspaceBox.geometry = edgeGeometry;
    teleoperationWorkspaceBox.position.copy(center);
    teleoperationWorkspaceSignature = signature;
  }
  teleoperationWorkspaceBox.userData.configured = true;
  teleoperationWorkspaceBox.visible = activeControlMode === "hand";
}

function createCubeWorkAreaVisual() {
  const root = new THREE.Group();
  root.name = "cube-work-area";
  root.visible = false;
  const fill = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({
      color: 0x56d7aa,
      transparent: true,
      opacity: 0.12,
      depthWrite: false,
      side: THREE.DoubleSide,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    }),
  );
  fill.renderOrder = 5;
  const outline = new THREE.LineLoop(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({
      color: 0x76f0c0,
      transparent: true,
      opacity: 0.94,
      depthTest: false,
      depthWrite: false,
    }),
  );
  outline.renderOrder = 15;
  root.add(fill, outline);
  root.userData.calibrationRole = "cube-work-area";
  return { root, fill, outline };
}

const cubeWorkAreaVisual = createCubeWorkAreaVisual();
scene.add(cubeWorkAreaVisual.root);

function createTableEdgeGuideVisual() {
  const root = new THREE.Group();
  root.name = "table-edge-guide-preview";
  root.visible = false;
  const line = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({
      color: 0xf5c46b,
      transparent: true,
      opacity: 0.95,
      depthTest: false,
      depthWrite: false,
    }),
  );
  line.renderOrder = 22;
  const markers = Array.from({ length: 3 }, (_, index) => {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(index === 1 ? 0.014 : 0.011, 20, 14),
      new THREE.MeshBasicMaterial({
        color: index === 1 ? 0xffffff : 0xf5c46b,
        depthTest: false,
        depthWrite: false,
      }),
    );
    marker.renderOrder = 24;
    root.add(marker);
    return marker;
  });
  root.add(line);
  root.userData.calibrationRole = "table-edge-guide-preview";
  return { root, line, markers };
}

const tableEdgeGuideVisual = createTableEdgeGuideVisual();
scene.add(tableEdgeGuideVisual.root);

function handleTableEdgeGuidePreview(detail) {
  const points = detail?.base_points_m;
  const valid = detail?.active === true
    && Array.isArray(points) && points.length === 3
    && points.every((point) => Array.isArray(point) && point.length === 3
      && point.every((component) => Number.isFinite(Number(component))));
  if (!valid) {
    tableEdgeGuideVisual.root.visible = false;
    return;
  }
  const vectors = points.map(([x, y, z]) => new THREE.Vector3(
    Number(x),
    Number(y),
    Number(z) + 0.008,
  ));
  tableEdgeGuideVisual.line.geometry.dispose();
  tableEdgeGuideVisual.line.geometry = new THREE.BufferGeometry().setFromPoints(vectors);
  const dragIndex = Number.isInteger(detail.drag_index) ? detail.drag_index : -1;
  tableEdgeGuideVisual.markers.forEach((marker, index) => {
    marker.position.copy(vectors[index]);
    const active = index === dragIndex;
    marker.scale.setScalar(active ? 1.55 : 1);
    marker.material.color.setHex(active ? 0x54d9e8 : index === 1 ? 0xffffff : 0xf5c46b);
  });
  tableEdgeGuideVisual.root.visible = true;
}

function handleCubeWorkArea(detail) {
  const polygon = detail?.base_polygon_m;
  const valid = Array.isArray(polygon) && polygon.length >= 3
    && polygon.every((point) => Array.isArray(point) && point.length === 3
      && point.every((component) => Number.isFinite(Number(component))));
  if (!valid) {
    cubeWorkAreaVisual.root.visible = false;
    return;
  }
  const points = polygon.map((point) => point.map(Number));
  const shape = new THREE.Shape();
  shape.moveTo(points[0][0], points[0][1]);
  points.slice(1).forEach(([x, y]) => shape.lineTo(x, y));
  shape.closePath();
  cubeWorkAreaVisual.fill.geometry.dispose();
  cubeWorkAreaVisual.fill.geometry = new THREE.ShapeGeometry(shape);
  cubeWorkAreaVisual.fill.position.z = points.reduce((sum, point) => sum + point[2], 0)
    / points.length + 0.0015;
  cubeWorkAreaVisual.outline.geometry.dispose();
  cubeWorkAreaVisual.outline.geometry = new THREE.BufferGeometry().setFromPoints(
    points.map(([x, y, z]) => new THREE.Vector3(x, y, z + 0.003)),
  );
  cubeWorkAreaVisual.root.visible = true;
}

function createDetectedCube() {
  const root = new THREE.Group();
  root.name = "rgb-detected-cube";
  root.visible = false;
  const material = new THREE.MeshPhysicalMaterial({
    color: 0x202729,
    emissive: 0x050707,
    emissiveIntensity: 0.22,
    roughness: 0.48,
    metalness: 0.04,
    clearcoat: 0.18,
    clearcoatRoughness: 0.5,
    transparent: true,
    // Tracking candidates are solid.  Opacity is reduced only for an
    // explicitly uncertain state such as a short occlusion.
    opacity: 1.0,
  });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.03), material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  root.add(mesh);
  const outlineMaterial = new THREE.LineBasicMaterial({
    color: 0x76f0c0,
    transparent: true,
    opacity: 0.82,
    depthTest: false,
    depthWrite: false,
  });
  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(mesh.geometry, 24),
    outlineMaterial,
  );
  outline.renderOrder = 8;
  root.add(outline);
  const selectedOutlineMaterial = new THREE.LineBasicMaterial({
    color: 0xfff2a8,
    transparent: true,
    opacity: 0.96,
    depthTest: false,
    depthWrite: false,
  });
  const selectedOutline = new THREE.LineSegments(
    new THREE.EdgesGeometry(mesh.geometry, 24),
    selectedOutlineMaterial,
  );
  // WebGL line width is not portable.  A slightly expanded second outline
  // gives selection a visibly heavier border on all browsers.
  selectedOutline.scale.setScalar(1.045);
  selectedOutline.renderOrder = 9;
  selectedOutline.visible = false;
  root.add(selectedOutline);
  const candidateOutlineMaterial = new THREE.LineDashedMaterial({
    color: 0x54d9e8,
    transparent: true,
    opacity: 0.58,
    dashSize: 0.006,
    gapSize: 0.004,
    depthTest: false,
    depthWrite: false,
  });
  root.renderOrder = 20;
  const candidateOutline = new THREE.LineSegments(
    new THREE.EdgesGeometry(mesh.geometry, 24),
    candidateOutlineMaterial,
  );
  candidateOutline.computeLineDistances();
  candidateOutline.renderOrder = 8;
  candidateOutline.visible = false;
  root.add(candidateOutline);
  const heading = new THREE.Mesh(
    new THREE.BoxGeometry(0.012, 0.0015, 0.001),
    new THREE.MeshBasicMaterial({
      color: 0x76f0c0,
      transparent: true,
      opacity: 0.9,
    }),
  );
  heading.position.set(0.006, 0, 0.0156);
  root.add(heading);
  root.userData.calibrationRole = "rgb-cube-observation";
  return {
    root,
    material,
    outline,
    outlineMaterial,
    selectedOutline,
    selectedOutlineMaterial,
    candidateOutline,
    candidateOutlineMaterial,
    heading,
    headingMaterial: heading.material,
    targetPosition: new THREE.Vector3(),
    targetQuaternion: new THREE.Quaternion(),
    targetOpacity: 0,
    lastUpdateMs: -Infinity,
    selected: false,
  };
}

const detectedCubes = new Map();
let selectedDetectedCubeId = null;
let cubePickVisualPinned = false;
let cubePickVisualHoldUntilMs = -Infinity;
let cubeMotionAction = null;
let carriedCubeId = null;
let carriedCubeAttached = false;
let carriedCubeReleasedFromGripper = false;
let carriedCubeHoldUntilMs = -Infinity;
const carriedCubeTcpOffset = new THREE.Vector3();
const carriedCubeTcpQuaternion = new THREE.Quaternion();

function detectedCubeFor(candidateId) {
  if (!detectedCubes.has(candidateId)) {
    const cube = createDetectedCube();
    cube.root.name = `rgb-detected-${candidateId}`;
    detectedCubes.set(candidateId, cube);
    scene.add(cube.root);
  }
  return detectedCubes.get(candidateId);
}

function selectedDetectedCube() {
  return selectedDetectedCubeId === null ? null : detectedCubes.get(selectedDetectedCubeId);
}

function detectedCubeCandidateAtPointer() {
  const targets = [...detectedCubes.values()]
    .filter((cube) => cube.root.visible)
    .map((cube) => cube.root);
  if (targets.length === 0) return null;
  const hits = raycaster.intersectObjects(targets, true);
  for (const hit of hits) {
    let object = hit.object;
    while (object) {
      const candidateId = object.userData.cubeCandidateId;
      if (typeof candidateId === "string") {
        const cube = detectedCubes.get(candidateId);
        const fresh = cube && performance.now() - cube.lastUpdateMs <= 1000;
        if (object.userData.cubeCandidateSelectable === true && fresh) {
          return candidateId;
        }
        break;
      }
      object = object.parent;
    }
  }
  return null;
}

function updateCubeSelectionCursor(event) {
  if (cartesianDrag || ringDrag || targetPoseEditor?.drag
      || directKeyHeld || cartesianPlaneKey) {
    elements.canvas.classList.remove("cube-selectable");
    return;
  }
  pointerToNdc(event);
  elements.canvas.classList.toggle(
    "cube-selectable",
    detectedCubeCandidateAtPointer() !== null,
  );
}

function tcpVisualTransform() {
  if (!finitePose(armState?.eef_pose_6d)) return null;
  const pose = armState.eef_pose_6d.map(Number);
  return {
    position: new THREE.Vector3(...pose.slice(0, 3)),
    quaternion: quaternionFromRpy(pose.slice(3, 6)),
  };
}

function matrixFromRowMajorTransform(value) {
  if (!Array.isArray(value) || value.length !== 4
      || !value.every((row) => Array.isArray(row) && row.length === 4
        && row.every((component) => Number.isFinite(Number(component))))) {
    return null;
  }
  const matrix = new THREE.Matrix4();
  matrix.set(
    ...value.flat().map(Number),
  );
  return matrix;
}

function refreshHandeyeBoardPreview() {
  const target = handeyeBoardCameraTarget;
  const tcp = tcpVisualTransform();
  if (!target || !tcp) {
    handeyeBoardPreview.visible = false;
    return;
  }
  const cameraToViewer = OPENCV_CAMERA_TO_VIEWER;
  const viewerTarget = cameraToViewer.clone()
    .multiply(target)
    .multiply(cameraToViewer.clone().invert());
  const localPosition = new THREE.Vector3();
  const localQuaternion = new THREE.Quaternion();
  const localScale = new THREE.Vector3();
  viewerTarget.decompose(localPosition, localQuaternion, localScale);
  handeyeBoardPreview.position.copy(localPosition)
    .applyQuaternion(tcp.quaternion)
    .add(tcp.position);
  handeyeBoardPreview.quaternion.copy(tcp.quaternion).multiply(localQuaternion);
  handeyeBoardPreview.visible = true;
}

function setHandeyeBoardObservation(preview) {
  handeyeBoardCameraTarget = matrixFromRowMajorTransform(preview?.target_pose_camera);
  refreshHandeyeBoardPreview();
}

function carriedDetectedCube({ create = false } = {}) {
  if (carriedCubeId !== null && detectedCubes.has(carriedCubeId)) {
    return detectedCubes.get(carriedCubeId);
  }
  if (!create) return null;
  carriedCubeId = carriedCubeId ?? selectedDetectedCubeId ?? "carried-cube";
  return detectedCubeFor(carriedCubeId);
}

function attachCarriedCube({ force = false } = {}) {
  if (carriedCubeAttached) return true;
  const cube = carriedDetectedCube({ create: force });
  const tcp = tcpVisualTransform();
  if (!cube || !tcp) return false;
  const separation = cube.root.position.distanceTo(tcp.position);
  if (!force && separation > 0.12) return false;
  const inverseTcp = tcp.quaternion.clone().invert();
  carriedCubeTcpOffset.copy(cube.root.position).sub(tcp.position).applyQuaternion(inverseTcp);
  if (force && separation > 0.12) carriedCubeTcpOffset.set(0, 0, 0);
  carriedCubeTcpQuaternion.copy(inverseTcp).multiply(cube.root.quaternion);
  carriedCubeAttached = true;
  carriedCubeReleasedFromGripper = false;
  carriedCubeHoldUntilMs = Infinity;
  cube.root.visible = true;
  cube.targetOpacity = 1.0;
  cube.outline.visible = true;
  cube.selectedOutline.visible = true;
  return true;
}

function updateCarriedCubePose() {
  if (!carriedCubeAttached) return;
  const cube = carriedDetectedCube();
  const tcp = tcpVisualTransform();
  if (!cube || !tcp) return;
  cube.targetPosition.copy(carriedCubeTcpOffset)
    .applyQuaternion(tcp.quaternion)
    .add(tcp.position);
  cube.targetQuaternion.copy(tcp.quaternion).multiply(carriedCubeTcpQuaternion);
  cube.root.position.copy(cube.targetPosition);
  cube.root.quaternion.copy(cube.targetQuaternion);
  cube.targetOpacity = 1.0;
  cube.root.visible = true;
}

function detachCarriedCube() {
  if (!carriedCubeAttached) return;
  updateCarriedCubePose();
  const cube = carriedDetectedCube();
  if (cube) {
    cube.root.position.copy(cube.targetPosition);
    cube.root.quaternion.copy(cube.targetQuaternion);
    cube.targetOpacity = 1.0;
    cube.lastUpdateMs = performance.now();
  }
  carriedCubeAttached = false;
  carriedCubeReleasedFromGripper = true;
  carriedCubeHoldUntilMs = performance.now() + 5000;
}

const calibrationEnvironment = createCalibrationEnvironment();
scene.add(calibrationEnvironment);
const calibrationTable = calibrationEnvironment.userData.calibrationTable;
let tableCalibrationState = loadTableCalibration();
let tableCalibrationEditing = false;
applyTableCalibration(tableCalibrationState, { persist: false, syncInputs: true });

const calibrationCamera = createCalibrationCamera();
scene.add(calibrationCamera.opticalCamera, calibrationCamera.helper);
let cameraCalibrationState = loadCameraCalibration();
let cameraCalibrationEditing = false;
let cameraCalibrationViewActive = false;
let handeyeCameraTransform = null;
let handeyeD405Transform = null;
applyCameraCalibration(cameraCalibrationState, { persist: false, syncInputs: true });
window.arx5GetSceneCalibration = () => ({
  camera: {
    position_mm: [
      Number(cameraCalibrationState.xMm),
      Number(cameraCalibrationState.yMm),
      Number(cameraCalibrationState.zMm),
    ],
    roll_deg: Number(cameraCalibrationState.rollDeg),
    pitch_deg: Number(cameraCalibrationState.pitchDeg),
    yaw_deg: Number(cameraCalibrationState.yawDeg),
  },
  table: {
    position_mm: [
      Number(tableCalibrationState.xMm),
      Number(tableCalibrationState.yMm),
      Number(tableCalibrationState.zMm),
    ],
    size_mm: [
      Number(tableCalibrationState.widthMm),
      Number(tableCalibrationState.depthMm),
    ],
    yaw_deg: Number(tableCalibrationState.yawDeg),
  },
});

function handleCubeDetection(detection) {
  const now = performance.now();
  const candidates = Array.isArray(detection?.candidates) && detection.candidates.length > 0
    ? detection.candidates
    : [{ ...detection, candidate_id: "cube-primary", selected: true }];
  const seen = new Set();
  selectedDetectedCubeId = detection?.selected_candidate_id ?? null;
  candidates.forEach((candidate) => {
    const pose = candidate?.pose_base;
    const candidateId = String(candidate?.candidate_id ?? "cube-primary");
    if (!Array.isArray(pose?.base_xyz_m)) return;
    const [x, y, z] = pose.base_xyz_m.map(Number);
    const yaw = Number(pose.yaw_rad);
    if (![x, y, z, yaw].every(Number.isFinite)) return;
    const cube = detectedCubeFor(candidateId);
    const selected = candidate.selected === true || candidateId === selectedDetectedCubeId;
    const status = String(candidate.status ?? "tracking");
    const ignored = status === "ignored";
    cube.root.userData.cubeCandidateId = candidateId;
    cube.root.userData.cubeCandidateSelectable = (
      status === "tracking" && !(carriedCubeAttached && candidateId === carriedCubeId)
    );
    const colorHex = {
      black: 0x202729,
      white: 0xf1f4ee,
      blue: 0x3d8dff,
    }[String(candidate.color ?? "")] ?? 0x202729;
    cube.material.color.setHex(colorHex);
    cube.material.emissive.setHex(colorHex === 0x202729 ? 0x050707 : 0x07101c);
    cube.selected = selected;
    cube.lastUpdateMs = now;
    if (carriedCubeAttached && candidateId === carriedCubeId) {
      cube.targetOpacity = 1.0;
      cube.outline.visible = true;
      cube.selectedOutline.visible = selected;
      cube.candidateOutline.visible = false;
      cube.heading.visible = true;
      seen.add(candidateId);
      return;
    }
    cube.targetPosition.set(x, y, z);
    cube.targetQuaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
    const uncertain = status === "occluded";
    cube.targetOpacity = ignored ? 0.30 : uncertain ? 0.48 : 1.0;
    cube.outline.visible = selected;
    cube.selectedOutline.visible = selected;
    cube.candidateOutline.visible = !selected;
    cube.heading.visible = selected;
    if (!cube.root.visible) {
      cube.root.position.copy(cube.targetPosition);
      cube.root.quaternion.copy(cube.targetQuaternion);
      cube.root.visible = true;
    }
    seen.add(candidateId);
    if (selected) selectedDetectedCubeId = candidateId;
  });
  for (const [candidateId, cube] of detectedCubes) {
    if (seen.has(candidateId)) continue;
    const held = candidateId === selectedDetectedCubeId
      && (cubePickVisualPinned || now < cubePickVisualHoldUntilMs);
    const carried = candidateId === carriedCubeId
      && (carriedCubeAttached || now < carriedCubeHoldUntilMs);
    // Let render() apply the 1.8 s stale timeout.  A single dropped detector
    // response should retain the last pose instead of blinking the cube out.
    if (!held && !carried) {
      if (candidateId !== selectedDetectedCubeId) cube.selected = false;
      cube.outline.visible = cube.selected;
      cube.selectedOutline.visible = cube.selected;
    }
  }
}

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const dragPlane = new THREE.Plane();
const dragPoint = new THREE.Vector3();
const dragOrigin = new THREE.Vector3();
const dragAxis = new THREE.Vector3();
const dragBasisX = new THREE.Vector3();
const dragBasisY = new THREE.Vector3();

let actualModel;
let previewModel;
let selectionRing;
let eefDragHandle;
let cartesianTargetMarker;
let cartesianGuideLine;
let plannedPathLine;
let plannedPathTube;
let executedPathLine;
let executedPathTube;
let plannedPathWaypoints;
let plannedPathFlowLine;
let plannedPathCurve = null;
let plannedPathFlowCurve = null;
let plannedPathSamples = [];
let executedPathFeedbackPoints = [];
let executedPathGeometryDirty = false;
let lastExecutedPathGeometryUpdateAt = -Infinity;
let plannedPathExecutionActive = false;
let plannedPathExecutionProgress = 0;
let plannedPathFeedbackDirty = false;
let renderedPathProgress = -1;
let lastPathSplitGeometryUpdateAt = -Infinity;
let plannedPathFlowStartProgress = 0;
let lastPlannedPathFlowUpdateAt = -Infinity;
let lastArmStateUiUpdateAt = -Infinity;
let plannedPathMode = null;
let targetPoseEditor;
let targetGripperGhost;
let cubePickGraspGhost;
let ringHitTargets = [];
let selectedJoint = 0;
let armState = getArmState();
let actualAngles = DISCONNECTED_DISPLAY_ANGLES.slice();
let displayedAngles = actualAngles.slice();
let previewAngles = actualAngles.slice();
let previewDirty = false;
let previewKind = null;
let cartesianPreview = null;
let activeControlMode = "xyz";
let activeXyzPathMode = "smooth";
let modelsReady = false;
let simulationRendererAvailable = true;
let simulationReloading = false;
let simulationRecoveryTimer;
let simulationRestoreTimer;
let commandPending = false;
let dialDrag;
let ringDrag;
let cartesianDrag;
let canvasPointerDown;
let canvasHovered = false;
let directKeyHeld = false;
let tcpControlsVisible = true;
let cartesianPlaneKey = null;
let livePreviewInFlight = false;
let livePreviewQueued = false;
let livePreviewTimer;
let lastLivePreviewAt = 0;
let livePreviewRequestId = null;
let livePreviewDragGeneration = null;
let livePreviewDragSource = null;
let servoHeartbeatTimer;
let gripperKeyHeartbeatTimer;
let gripperServoDirection = 0;
const gripperKeysHeld = new Set();
let cartesianDragSequence = 0;
let targetEditorLivePreview = null;
let targetEditorLivePreviewSequence = 0;
let completedTargetEditorLivePreview = null;
let formalPreviewFallbackDelta = null;
let formalPreviewDragGeneration = null;
let formalPreviewDragSource = null;
let formalPreviewTargetSignature = null;
let displayedPreviewAngles = previewAngles.slice();
let actualGripperWidthMm = DISCONNECTED_GRIPPER_WIDTH_MM;
let displayedGripperWidthMm = actualGripperWidthMm;
let targetGripperWidthMm = actualGripperWidthMm;
let displayedTargetGripperWidthMm = targetGripperWidthMm;
let targetPose = null;
let parkControlFollowing = false;
let lastRenderTime = performance.now();

function setPill(text, stateName) {
  const label = elements.modelStatus.querySelector(":scope > span");
  if (label) label.textContent = text;
  else elements.modelStatus.textContent = text;
  elements.modelStatus.title = text;
  elements.modelStatus.setAttribute("aria-label", text);
  elements.modelStatus.className = `pill ${stateName}`;
}

function finiteJointVector(value) {
  return Array.isArray(value)
    && value.length === 6
    && value.every((item) => Number.isFinite(item));
}

function finitePose(value) {
  return Array.isArray(value)
    && value.length === 6
    && value.every((item) => Number.isFinite(item));
}

function formatXyz(valuesMm) {
  if (!Array.isArray(valuesMm) || valuesMm.length !== 3
      || !valuesMm.every(Number.isFinite)) return "—";
  return valuesMm.map((value) => value.toFixed(1)).join(" · ");
}

function actualTargetPoseFromState(state = armState) {
  if (!state.connected || !finitePose(state.eef_pose_6d)) return null;
  return {
    position: state.eef_pose_6d.slice(0, 3).map(Number),
    orientation: quaternionToArray(quaternionFromRpy(
      state.eef_pose_6d.slice(3, 6).map(Number),
    )),
    ...(Number.isFinite(state.gripper_position_mm)
      ? { gripperWidth: Number(state.gripper_position_mm) / 1000.0 }
      : {}),
  };
}

function beginMotionFeedbackFollow() {
  parkControlFollowing = true;
  const pose = actualTargetPoseFromState();
  if (pose) targetPoseEditor?.beginFeedbackFollow(pose);
  updateCartesianInspector();
}

function endMotionFeedbackFollow() {
  parkControlFollowing = false;
  const pose = actualTargetPoseFromState();
  targetPoseEditor?.endFeedbackFollow();
  if (pose) {
    targetPose = cloneTargetPose(pose);
    targetPoseEditor?.setTargetPose(targetPose, { markReady: false });
    updateTargetPoseReadout();
  }
  updateCartesianInspector();
}

function targetPositionMmFromInputs() {
  return [
    elements.xyzInputs.x.valueAsNumber,
    elements.xyzInputs.y.valueAsNumber,
    elements.xyzInputs.z.valueAsNumber,
  ];
}

function requestedCartesianDelta() {
  const currentMm = finitePose(armState.eef_pose_6d)
    ? armState.eef_pose_6d.slice(0, 3).map((value) => Number(value) * 1000.0)
    : [Number.NaN, Number.NaN, Number.NaN];
  const targetMm = targetPositionMmFromInputs();
  return {
    x_mm: targetMm[0] - currentMm[0],
    y_mm: targetMm[1] - currentMm[1],
    z_mm: targetMm[2] - currentMm[2],
    orientation_xyzw: targetPose?.orientation?.map(Number),
    ...getMotionSpeedParams(),
    path_mode: activeXyzPathMode,
  };
}

function cartesianTargetSignature() {
  if (!targetPose) return null;
  return JSON.stringify([
    ...targetPose.position.map(Number),
    ...targetPose.orientation.map(Number),
    activeXyzPathMode,
  ]);
}

function xyzPathModeLabel() {
  return activeXyzPathMode === "smooth" ? "平滑曲线" : "末端直线";
}

function syncMotionSpeedUi() {
  const speed = getMotionSpeedPercent();
  const ratio = getMotionSpeedProgress();
  const tier = speed <= 35
    ? "precise"
    : speed <= 60
      ? "smooth"
      : speed <= 80
        ? "fast"
        : "max";
  elements.speedValue.textContent = `${speed.toFixed(0)}%`;
  elements.speedControl.dataset.speedTier = tier;
  elements.speedControl.style.setProperty(
    "--speed-fill",
    `${ratio * 100}%`,
  );
  elements.speedControl.style.setProperty(
    "--speed-pulse-duration",
    `${(2.4 - ratio * 1.45).toFixed(2)}s`,
  );
  elements.speedControl.style.setProperty("--speed-energy", ratio.toFixed(3));
}

function updateCartesianInputLimits() {
  Object.values(elements.xyzInputs).forEach((input) => {
    input.removeAttribute("min");
    input.removeAttribute("max");
  });
}

function validCartesianDelta(delta = requestedCartesianDelta()) {
  const values = [delta.x_mm, delta.y_mm, delta.z_mm];
  if (!values.every(Number.isFinite)) return false;
  if (Math.hypot(...values) >= 0.1) return true;
  const actualPose = actualTargetPoseFromState();
  if (!actualPose || !Array.isArray(delta.orientation_xyzw)) return false;
  const actualQuaternion = quaternionFromArray(actualPose.orientation);
  const targetQuaternion = quaternionFromArray(delta.orientation_xyzw);
  return 2 * Math.acos(THREE.MathUtils.clamp(
    Math.abs(actualQuaternion.dot(targetQuaternion)),
    -1,
    1,
  )) >= THREE.MathUtils.degToRad(0.05);
}

function hasCartesianVisualPreview() {
  return previewKind === "cartesian"
    || previewKind === "cartesian-live"
    || previewKind === "cartesian-limit"
    || previewKind === "cartesian-servo"
    || previewKind === "cartesian-servo-limit";
}

function hasTeleoperationPreview() {
  return previewKind === "teleoperation"
    || previewKind === "teleoperation-limit";
}

function hasFullRobotPreview() {
  return hasCartesianVisualPreview()
    || hasTeleoperationPreview()
    || previewKind === "park";
}

function currentCartesianPlane() {
  return Object.values(CARTESIAN_PLANES_BY_CODE)
    .find((plane) => plane.key === cartesianPlaneKey) ?? null;
}

function activeReadOnlyPreviewDrag() {
  if (targetEditorLivePreview) {
    return { source: "target-editor", session: targetEditorLivePreview };
  }
  if (cartesianDrag && !cartesianDrag.liveServo) {
    return { source: "cartesian-plane", session: cartesianDrag };
  }
  return null;
}

function flushLiveCartesianPreview() {
  livePreviewTimer = undefined;
  const activeDrag = activeReadOnlyPreviewDrag();
  if (!activeDrag || !livePreviewQueued || livePreviewInFlight) return;
  const delta = requestedCartesianDelta();
  if (!validCartesianDelta(delta)) {
    livePreviewQueued = false;
    return;
  }
  livePreviewQueued = false;
  livePreviewInFlight = true;
  lastLivePreviewAt = performance.now();
  livePreviewDragGeneration = activeDrag.session.generation;
  livePreviewDragSource = activeDrag.source;
  livePreviewRequestId = sendCommand("preview_xyz_live", delta);
  if (livePreviewRequestId === null) {
    livePreviewInFlight = false;
    livePreviewDragGeneration = null;
    livePreviewDragSource = null;
  }
}

function scheduleLiveCartesianPreview() {
  if (!activeReadOnlyPreviewDrag()) return;
  livePreviewQueued = true;
  if (livePreviewInFlight) return;
  const delay = Math.max(
    0,
    LIVE_PREVIEW_INTERVAL_MS - (performance.now() - lastLivePreviewAt),
  );
  if (livePreviewTimer) clearTimeout(livePreviewTimer);
  livePreviewTimer = setTimeout(flushLiveCartesianPreview, delay);
}

function clearQueuedLiveCartesianPreview() {
  livePreviewQueued = false;
  if (livePreviewTimer) {
    clearTimeout(livePreviewTimer);
    livePreviewTimer = undefined;
  }
}

function beginTargetEditorLivePreview() {
  completedTargetEditorLivePreview = null;
  if (!modelsReady || !motionAvailable()) return;
  if (previewDirty) resetPreview({ quiet: true });
  targetEditorLivePreview = {
    generation: ++targetEditorLivePreviewSequence,
    lastValidDelta: null,
    startAngles: actualAngles.slice(),
    previewInitialized: false,
  };
  previewKind = "cartesian-live";
  previewDirty = true;
  cartesianPreview = null;
  previewAngles = actualAngles.slice();
  displayedPreviewAngles = previewAngles.slice();
  if (previewModel) {
    setModelAngles(previewModel, previewAngles);
    setModelGripperWidth(previewModel, actualGripperWidthMm);
  }
  updatePreviewVisibility();
  updateInspector();
  updateCartesianInspector();
}

function updateTargetEditorLivePreview() {
  if (!targetEditorLivePreview) return;
  scheduleLiveCartesianPreview();
}

function endTargetEditorLivePreview({ cancelled = false } = {}) {
  const finished = targetEditorLivePreview;
  targetEditorLivePreview = null;
  clearQueuedLiveCartesianPreview();
  completedTargetEditorLivePreview = !cancelled && finished
    ? {
      fallbackDelta: finished.lastValidDelta,
      generation: finished.generation,
      source: "target-editor",
    }
    : null;
  if (cancelled && hasCartesianVisualPreview()) resetPreview({ quiet: true });
}

function takeCompletedTargetEditorLivePreview() {
  const completed = completedTargetEditorLivePreview;
  completedTargetEditorLivePreview = null;
  return completed;
}

function motionAvailable() {
  return Boolean(armState.connected)
    && (armState.mode === "active" || armState.mode === "damping")
    && !armState.busy
    && !armState.servo_active
    && !commandPending;
}

function liveServoEnabled() {
  return Boolean(elements.servoEnabled.checked);
}

function sendLiveServoTarget() {
  if (!cartesianDrag?.liveServo) return null;
  return sendServoMessage("update", requestedCartesianDelta());
}

function stopServoHeartbeat() {
  if (servoHeartbeatTimer) {
    clearInterval(servoHeartbeatTimer);
    servoHeartbeatTimer = undefined;
  }
}

function startServoHeartbeat() {
  stopServoHeartbeat();
  sendLiveServoTarget();
  servoHeartbeatTimer = setInterval(
    sendLiveServoTarget,
    SERVO_HEARTBEAT_INTERVAL_MS,
  );
}

function safeTargetWindow() {
  const current = actualAngles[selectedJoint];
  const configuredMin = armState.joint_min_deg?.[selectedJoint];
  const configuredMax = armState.joint_max_deg?.[selectedJoint];
  if (![current, configuredMin, configuredMax].every(Number.isFinite)) return null;
  const minimum = configuredMin + LIMIT_MARGIN_DEG;
  const maximum = configuredMax - LIMIT_MARGIN_DEG;
  return { minimum, maximum };
}

function fallbackGeometry(index) {
  const { size, center } = FALLBACK_LINKS[index];
  const geometry = new THREE.BoxGeometry(...size);
  geometry.translate(...center);
  geometry.computeVertexNormals();
  return geometry;
}

function classifyOfficialGripperGeometry(geometry) {
  if (gripperGeometryClassifications.has(geometry)) {
    return gripperGeometryClassifications.get(geometry);
  }
  const position = geometry.getAttribute("position");
  if (!position || geometry.index || position.count % 3 !== 0) {
    gripperGeometryClassifications.set(geometry, null);
    return null;
  }
  const triangleCount = position.count / 3;
  const parents = new Int32Array(triangleCount);
  const sizes = new Int32Array(triangleCount);
  const owners = new Map();
  for (let triangle = 0; triangle < triangleCount; triangle += 1) {
    parents[triangle] = triangle;
    sizes[triangle] = 1;
  }
  const find = (value) => {
    let root = value;
    while (parents[root] !== root) root = parents[root];
    while (parents[value] !== value) {
      const next = parents[value];
      parents[value] = root;
      value = next;
    }
    return root;
  };
  const union = (first, second) => {
    let a = find(first);
    let b = find(second);
    if (a === b) return;
    if (sizes[a] < sizes[b]) [a, b] = [b, a];
    parents[b] = a;
    sizes[a] += sizes[b];
  };
  for (let vertex = 0; vertex < position.count; vertex += 1) {
    const key = `${Math.round(position.getX(vertex) * 1e6)},${Math.round(position.getY(vertex) * 1e6)},${Math.round(position.getZ(vertex) * 1e6)}`;
    const triangle = Math.floor(vertex / 3);
    const owner = owners.get(key);
    if (owner === undefined) owners.set(key, triangle);
    else union(triangle, owner);
  }
  const bounds = new Map();
  for (let vertex = 0; vertex < position.count; vertex += 1) {
    const root = find(Math.floor(vertex / 3));
    let bound = bounds.get(root);
    if (!bound) {
      bound = {
        minX: Infinity,
        minY: Infinity,
        maxX: -Infinity,
        maxY: -Infinity,
      };
      bounds.set(root, bound);
    }
    const x = position.getX(vertex);
    const y = position.getY(vertex);
    bound.minX = Math.min(bound.minX, x);
    bound.minY = Math.min(bound.minY, y);
    bound.maxX = Math.max(bound.maxX, x);
    bound.maxY = Math.max(bound.maxY, y);
  }
  const componentSides = new Map();
  let positiveInnerY = Infinity;
  let negativeInnerY = -Infinity;
  bounds.forEach((bound, root) => {
    if (bound.minX < 0.055 || bound.maxX < 0.065) return;
    if (bound.minY > 0.025) {
      componentSides.set(root, 1);
      positiveInnerY = Math.min(positiveInnerY, bound.minY);
    } else if (bound.maxY < -0.025) {
      componentSides.set(root, -1);
      negativeInnerY = Math.max(negativeInnerY, bound.maxY);
    }
  });
  const sides = new Int8Array(position.count);
  let movableVertexCount = 0;
  for (let vertex = 0; vertex < position.count; vertex += 1) {
    const side = componentSides.get(find(Math.floor(vertex / 3))) ?? 0;
    sides[vertex] = side;
    if (side) movableVertexCount += 1;
  }
  const classification = movableVertexCount > 0
    && Number.isFinite(positiveInnerY)
    && Number.isFinite(negativeInnerY)
    ? { sides, positiveInnerY, negativeInnerY, movableVertexCount }
    : null;
  gripperGeometryClassifications.set(geometry, classification);
  return classification;
}

function createDynamicGripperGeometry(sourceGeometry) {
  const classification = classifyOfficialGripperGeometry(sourceGeometry);
  if (!classification) return { geometry: sourceGeometry, dynamic: false };
  const geometry = sourceGeometry.clone();
  const position = geometry.getAttribute("position");
  const movableVertices = [];
  for (let vertex = 0; vertex < position.count; vertex += 1) {
    if (classification.sides[vertex]) movableVertices.push(vertex);
  }
  gripperGeometryDeformations.set(geometry, {
    baseY: Float32Array.from(movableVertices, (vertex) => position.getY(vertex)),
    movableVertices: Uint32Array.from(movableVertices),
    sides: classification.sides,
    positiveInnerY: classification.positiveInnerY,
    negativeInnerY: classification.negativeInnerY,
  });
  return { geometry, dynamic: true };
}

function setOfficialGripperWidth(geometry, widthMm, maximumMm) {
  const deformation = gripperGeometryDeformations.get(geometry);
  if (!deformation) return false;
  const openingM = THREE.MathUtils.clamp(widthMm, 0, maximumMm) / 1000.0;
  const positiveShift = openingM / 2 - deformation.positiveInnerY;
  const negativeShift = -openingM / 2 - deformation.negativeInnerY;
  const position = geometry.getAttribute("position");
  for (let index = 0; index < deformation.movableVertices.length; index += 1) {
    const vertex = deformation.movableVertices[index];
    const side = deformation.sides[vertex];
    position.setY(
      vertex,
      deformation.baseY[index] + (side > 0 ? positiveShift : negativeShift),
    );
  }
  position.needsUpdate = true;
  return true;
}

function prepareLinkGeometry(buffer) {
  const loader = new STLLoader();
  const geometry = loader.parse(buffer);
  // Binary STL already contains one normal per facet. Recomputing hundreds
  // of thousands of normals on every page load only delays first render.
  if (!geometry.getAttribute("normal")) geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

async function openModelCache() {
  if (!("caches" in globalThis)) return null;
  try {
    return await globalThis.caches.open(MODEL_CACHE_NAME);
  } catch (error) {
    console.warn("X5 model cache unavailable; using normal browser cache", error);
    return null;
  }
}

async function loadLinkGeometry(index, cache = null) {
  const filename = index === 0 ? "base_link.STL" : `link${index}.STL`;
  const url = MODEL_URLS[index];
  if (cache) {
    const cachedResponse = await cache.match(url);
    if (cachedResponse) {
      try {
        return {
          geometry: prepareLinkGeometry(await cachedResponse.arrayBuffer()),
          cacheHit: true,
        };
      } catch (error) {
        console.warn(`Cached X5 mesh ${filename} is invalid; downloading again`, error);
        await cache.delete(url).catch(() => {});
      }
    }
  }

  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), MODEL_FETCH_TIMEOUT_MS);
  let response;
  let buffer;
  try {
    response = await fetch(url, {
      credentials: "same-origin",
      cache: "force-cache",
      signal: controller.signal,
    });
    if (!response.ok) throw new Error(`HTTP ${response.status} while loading ${filename}`);
    const cacheResponse = cache ? response.clone() : null;
    buffer = await response.arrayBuffer();
    if (cache && cacheResponse) {
      void cache.put(url, cacheResponse).catch((error) => {
        console.warn(`Unable to persist X5 mesh ${filename}`, error);
      });
    }
  } finally {
    window.clearTimeout(timeout);
  }
  const geometry = prepareLinkGeometry(buffer);
  return { geometry, cacheHit: false };
}

async function loadD405Geometry() {
  const response = await fetch(D405_MODEL_URL, {
    credentials: "same-origin",
    cache: "force-cache",
  });
  if (!response.ok) {
    throw new Error(`HTTP ${response.status} while loading Intel RealSense D405`);
  }
  return prepareLinkGeometry(await response.arrayBuffer());
}

async function loadGeometries(onLinkReady = () => {}) {
  const modelCache = await openModelCache();
  const d405Request = Promise.allSettled([loadD405Geometry()]);
  let completed = 0;
  let cacheHits = 0;
  const total = FALLBACK_LINKS.length;
  const requests = FALLBACK_LINKS.map((_, index) => loadLinkGeometry(index, modelCache)
    .then(({ geometry, cacheHit }) => {
      completed += 1;
      if (cacheHit) cacheHits += 1;
      onLinkReady(index);
      elements.loadingText.textContent = modelCache
        ? `正在加载官方 X5 模型… ${completed}/${total} · 缓存 ${cacheHits}`
        : `正在加载官方 X5 模型… ${completed}/${total}`;
      return geometry;
    }));
  const results = await Promise.allSettled(requests);
  let degraded = false;
  const geometries = results.map((result, index) => {
    if (result.status === "fulfilled") return result.value;
    degraded = true;
    console.warn(`X5 link ${index} mesh unavailable; using fallback`, result.reason);
    return fallbackGeometry(index);
  });
  const [d405Result] = await d405Request;
  const d405Geometry = d405Result.status === "fulfilled"
    ? d405Result.value
    : null;
  if (!d405Geometry) {
    console.warn(
      "Official Intel RealSense D405 mesh unavailable; using dimensional fallback",
      d405Result.reason,
    );
  }
  return {
    geometries,
    degraded,
    d405Geometry,
    d405Degraded: !d405Geometry,
    cacheEnabled: Boolean(modelCache),
    cacheHits,
  };
}

function createLinkMaterial(index, ghost) {
  const isJ3 = index === J3_LINK_MATERIAL_INDEX;
  if (ghost) {
    return new THREE.MeshStandardMaterial({
      color: isJ3 ? 0xb9c2c8 : 0x3c4247,
      emissive: isJ3 ? 0x5b6870 : 0x171b1e,
      emissiveIntensity: 0.34,
      metalness: isJ3 ? 0.72 : 0.1,
      roughness: isJ3 ? 0.3 : 0.62,
      transparent: true,
      opacity: 0.36,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
  }
  return new THREE.MeshStandardMaterial({
    color: isJ3 ? J3_METAL_COLOR : PLASTIC_BLACK_COLOR,
    emissive: 0x070807,
    emissiveIntensity: 0.08,
    metalness: isJ3 ? 0.86 : 0.08,
    roughness: isJ3 ? 0.25 : 0.68,
    side: THREE.DoubleSide,
  });
}

function createD405Material(ghost) {
  if (ghost) {
    return new THREE.MeshStandardMaterial({
      color: 0x3c4247,
      emissive: 0x171b1e,
      emissiveIntensity: 0.34,
      metalness: 0.55,
      roughness: 0.38,
      transparent: true,
      opacity: 0.36,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
  }
  return new THREE.MeshStandardMaterial({
    color: 0xbfc4c8,
    emissive: 0x090a0a,
    emissiveIntensity: 0.06,
    metalness: 0.84,
    roughness: 0.27,
    side: THREE.DoubleSide,
  });
}

function createD405Visual(geometry, ghost) {
  const eefRoot = new THREE.Group();
  eefRoot.name = ghost ? "preview-d405-eef-frame" : "actual-d405-eef-frame";
  eefRoot.position.x = EEF_OFFSET_LINK6_M;

  // T_eef_camera maps D405 optical-frame points into eef_link. Keep this
  // object matrix-controlled so the calibrated SE(3) can be copied without an
  // Euler conversion or a camera-axis convention shortcut.
  const opticalRoot = new THREE.Group();
  opticalRoot.name = ghost ? "preview-d405-color-optical-frame" : "actual-d405-color-optical-frame";
  opticalRoot.matrixAutoUpdate = false;
  opticalRoot.visible = false;
  eefRoot.add(opticalRoot);

  // Official URDF: camera_link -> color_optical_frame uses the nominal optical
  // rotation below. Invert it because camera_link is rendered under the
  // calibrated optical frame.
  const cameraLink = new THREE.Group();
  cameraLink.name = ghost ? "preview-d405-camera-link" : "actual-d405-camera-link";
  const linkToOptical = new THREE.Matrix4().makeRotationFromEuler(
    new THREE.Euler(...D405_LINK_TO_OPTICAL_RPY_RAD, "XYZ"),
  );
  cameraLink.matrix.copy(linkToOptical).invert();
  cameraLink.matrixAutoUpdate = false;
  opticalRoot.add(cameraLink);

  const official = Boolean(geometry);
  const mesh = new THREE.Mesh(
    geometry ?? new RoundedBoxGeometry(...D405_BODY_SIZE_M, 3, 0.0025),
    createD405Material(ghost),
  );
  mesh.name = official
    ? (ghost ? "preview-intel-realsense-d405" : "actual-intel-realsense-d405")
    : (ghost ? "preview-d405-dimensional-fallback" : "actual-d405-dimensional-fallback");
  if (official) {
    mesh.position.set(...D405_LINK_TO_VISUAL_XYZ_M);
    mesh.rotation.set(...D405_LINK_TO_VISUAL_RPY_RAD, "XYZ");
    mesh.scale.setScalar(D405_STL_SCALE_M);
  } else {
    mesh.position.set(...D405_FALLBACK_BODY_XYZ_M);
  }
  mesh.userData.jointIndex = 5;
  mesh.userData.cameraModel = "Intel RealSense D405";
  mesh.userData.officialGeometry = official;
  mesh.renderOrder = ghost ? 4 : 2;
  mesh.castShadow = !ghost;
  mesh.receiveShadow = true;
  cameraLink.add(mesh);
  return { eefRoot, opticalRoot, mesh, official };
}

function createGripperVisual(ghost) {
  const root = new THREE.Group();
  root.name = ghost ? "preview-gripper-fingers" : "actual-gripper-fingers";
  const fingerRoots = [];
  const meshes = [];
  const bodyGeometry = new THREE.BoxGeometry(0.066, 0.009, 0.024);
  const tipGeometry = new THREE.BoxGeometry(0.022, GRIPPER_FINGER_THICKNESS_M, 0.032);
  const padGeometry = new THREE.BoxGeometry(0.020, 0.002, 0.026);

  [-1, 1].forEach((side) => {
    const fingerRoot = new THREE.Group();
    fingerRoot.name = side < 0 ? "gripper-finger-negative" : "gripper-finger-positive";

    const body = new THREE.Mesh(bodyGeometry, createLinkMaterial(6, ghost));
    body.position.set(0.116, 0, 0);
    body.userData.jointIndex = 5;
    body.renderOrder = ghost ? 4 : 2;
    fingerRoot.add(body);
    meshes.push(body);

    const tip = new THREE.Mesh(tipGeometry, createLinkMaterial(6, ghost));
    tip.position.set(0.145, 0, 0);
    tip.userData.jointIndex = 5;
    tip.renderOrder = ghost ? 4 : 2;
    fingerRoot.add(tip);
    meshes.push(tip);

    const padMaterial = ghost
      ? createLinkMaterial(6, true)
      : new THREE.MeshStandardMaterial({
        color: 0x55d7bb,
        emissive: 0x0b5b4c,
        emissiveIntensity: 0.42,
        metalness: 0.12,
        roughness: 0.62,
      });
    const pad = new THREE.Mesh(padGeometry, padMaterial);
    pad.position.set(
      0.145,
      -side * (GRIPPER_FINGER_THICKNESS_M / 2 - 0.001),
      0,
    );
    pad.userData.jointIndex = 5;
    pad.renderOrder = ghost ? 5 : 3;
    fingerRoot.add(pad);
    meshes.push(pad);

    root.add(fingerRoot);
    fingerRoots.push({ root: fingerRoot, side });
  });
  return { root, fingerRoots, meshes };
}

function setGripperVisualWidth(gripper, widthMm, maximumMm = 60) {
  if (!gripper || !Number.isFinite(widthMm)) return;
  const openingM = THREE.MathUtils.clamp(widthMm, 0, maximumMm) / 1000.0;
  const centerOffset = openingM / 2 + GRIPPER_FINGER_THICKNESS_M / 2;
  gripper.fingerRoots.forEach((finger) => {
    finger.root.position.y = finger.side * centerOffset;
  });
}

function createTargetGripperGhost() {
  const root = new THREE.Group();
  root.name = "target-gripper-ghost";
  const palmMaterial = createLinkMaterial(6, true);
  const palm = new THREE.Mesh(
    new THREE.CylinderGeometry(0.031, 0.031, 0.078, 24),
    palmMaterial,
  );
  palm.rotation.z = Math.PI / 2;
  palm.position.x = -0.105;
  palm.renderOrder = 10;
  root.add(palm);

  const gripper = createGripperVisual(true);
  gripper.root.position.x = -EEF_OFFSET_LINK6_M;
  root.add(gripper.root);
  root.visible = false;
  scene.add(root);
  return {
    root,
    gripper,
    meshes: [palm, ...gripper.meshes],
  };
}

function createCubePickGraspGhost() {
  const ghost = createTargetGripperGhost();
  ghost.root.name = "cube-pick-grasp-ghost";
  ghost.meshes.forEach((mesh) => {
    mesh.material.color.setHex(0x55e6a5);
    mesh.material.emissive.setHex(0x12634e);
    mesh.material.emissiveIntensity = 0.48;
    mesh.material.opacity = 0.42;
  });
  return ghost;
}

function setCubePickGraspGhost(preview = null) {
  if (!cubePickGraspGhost) return;
  const positionMm = preview?.grasp_pose_mm;
  const orientation = preview?.grasp_orientation_xyzw;
  const valid = Array.isArray(positionMm) && positionMm.length === 3
    && positionMm.every(Number.isFinite)
    && Array.isArray(orientation) && orientation.length === 4
    && orientation.every(Number.isFinite);
  if (!valid) {
    cubePickGraspGhost.root.visible = false;
    return;
  }
  cubePickGraspGhost.root.position.set(
    positionMm[0] / 1000,
    positionMm[1] / 1000,
    positionMm[2] / 1000,
  );
  cubePickGraspGhost.root.quaternion.copy(quaternionFromArray(orientation));
  setGripperVisualWidth(
    cubePickGraspGhost.gripper,
    Number(preview.grasp_width_mm) || 30,
    Number.isFinite(armState.gripper_max_mm) ? armState.gripper_max_mm : 60,
  );
  cubePickGraspGhost.root.visible = true;
  cubePickGraspGhost.root.updateMatrixWorld(true);
}

function setCubeStageTcpGhost(preview = null) {
  const pose = preview?.preview_tcp_pose;
  setCubePickGraspGhost(
    pose
      ? {
        grasp_pose_mm: pose.target_xyz_mm,
        grasp_orientation_xyzw: pose.orientation_xyzw,
        grasp_width_mm: actualGripperWidthMm,
      }
      : null,
  );
}

function setTargetGhostAppearance(state = "pending") {
  if (!targetGripperGhost) return;
  const editing = targetPoseEditor && [
    TARGET_EDIT_STATE.EDIT_POSITION,
    TARGET_EDIT_STATE.EDIT_ORIENTATION,
  ].includes(targetPoseEditor.state);
  const color = state === "valid" ? 0x79e0bf : state === "invalid" ? 0xff665f : 0xffb55e;
  const emissive = state === "valid" ? 0x12634e : state === "invalid" ? 0x8c1717 : 0x754000;
  targetGripperGhost.meshes.forEach((mesh) => {
    mesh.material.color.setHex(color);
    mesh.material.emissive.setHex(emissive);
    mesh.material.emissiveIntensity = editing ? 0.58 : 0.38;
    mesh.material.opacity = editing ? 0.52 : 0.34;
  });
}

function updateTargetGhost(pose = targetPose) {
  if (!targetGripperGhost) return;
  if (!armState.connected || !pose) {
    targetGripperGhost.root.visible = false;
    return;
  }
  targetGripperGhost.root.visible = activeControlMode === "xyz" && !parkControlFollowing;
  targetGripperGhost.root.position.fromArray(pose.position);
  targetGripperGhost.root.quaternion.copy(quaternionFromArray(pose.orientation));
  setGripperVisualWidth(
    targetGripperGhost.gripper,
    Number.isFinite(pose.gripperWidth)
      ? pose.gripperWidth * 1000.0
      : actualGripperWidthMm,
    Number.isFinite(armState.gripper_max_mm) ? armState.gripper_max_mm : 60,
  );
  targetGripperGhost.root.updateMatrixWorld(true);
}

function updateTargetPoseReadout({ writePosition = true } = {}) {
  if (!targetPose) return;
  if (writePosition) {
    const positionMm = targetPose.position.map((value) => value * 1000.0);
    elements.xyzInputs.x.value = positionMm[0].toFixed(1);
    elements.xyzInputs.y.value = positionMm[1].toFixed(1);
    elements.xyzInputs.z.value = positionMm[2].toFixed(1);
  }
  const quaternion = quaternionFromArray(targetPose.orientation);
  const approach = new THREE.Vector3(...LOCAL_APPROACH_AXIS)
    .applyQuaternion(quaternion)
    .normalize();
  ["x", "y", "z"].forEach((axis) => {
    elements.targetApproach[axis].textContent = approach[axis].toFixed(3);
  });
  elements.targetQuaternion.textContent = quaternionToArray(quaternion)
    .map((value) => value.toFixed(5))
    .join("  ");
  if (targetPoseEditor && document.activeElement !== elements.targetRoll) {
    elements.targetRoll.value = THREE.MathUtils.radToDeg(
      targetPoseEditor.rollRadians,
    ).toFixed(1);
  }
  updateTargetGhost(targetPose);
}

function applyEditedTargetPose(pose, reason = "external", { fromEditor = false } = {}) {
  targetPose = cloneTargetPose(pose);
  if (!targetPose) return;
  if (!fromEditor && targetPoseEditor) targetPoseEditor.setTargetPose(targetPose);
  updateTargetPoseReadout();
  setTargetGhostAppearance("pending");
  if (hasCartesianVisualPreview() && !targetEditorLivePreview) {
    resetPreview({ quiet: true });
  }
  updateCartesianInspector();
  if (["axis-drag", "rotation-drag", "roll-key"].includes(reason)) {
    setXyzStatus(
      targetEditorLivePreview && reason !== "roll-key"
        ? "目标正在变化；半透明机械臂通过只读 IK 连续跟随。"
        : "目标已修改，等待完成编辑后进行 IK 预览。",
      "ready",
    );
  }
}

function updateTargetEditingState({ state, label }) {
  elements.targetEditingState.textContent = label;
  elements.targetEditBadge.textContent = label;
  elements.targetEditBadge.hidden = state === TARGET_EDIT_STATE.IDLE;
  setTargetGhostAppearance(targetPoseEditor?.validationState ?? "pending");
  updateCartesianInspector();
}

function initializeTargetPoseEditing() {
  targetGripperGhost = createTargetGripperGhost();
  cubePickGraspGhost = createCubePickGraspGhost();
  targetPoseEditor = new TargetPoseEditor({
    scene,
    camera,
    controls,
    canvas: elements.canvas,
    onPoseChange: (pose, reason) => {
      applyEditedTargetPose(pose, reason, { fromEditor: true });
    },
    onDragStart: () => {
      beginTargetEditorLivePreview();
      setXyzStatus("正在拖动 TCP；半透明机械臂会连续显示实时 IK。", "ready");
    },
    onDragChange: () => updateTargetEditorLivePreview(),
    onDragEnd: (_pose, _mode, options) => {
      endTargetEditorLivePreview(options);
    },
    onStateChange: updateTargetEditingState,
    onTargetReady: (pose) => {
      applyEditedTargetPose(pose, "ready", { fromEditor: true });
      const livePreview = takeCompletedTargetEditorLivePreview();
      if (motionAvailable() && validCartesianDelta()) {
        requestCartesianPreview({
          fallbackDelta: livePreview?.fallbackDelta,
          dragGeneration: livePreview?.generation,
          dragSource: livePreview?.source,
        });
      } else {
        if (hasCartesianVisualPreview()) resetPreview({ quiet: true });
        setXyzStatus("目标已就绪；连接并处于可预览状态后再验证。", "ready");
      }
    },
  });
  const actualPose = actualTargetPoseFromState();
  if (actualPose) {
    targetPose = cloneTargetPose(actualPose);
    targetPoseEditor.setActualPose(actualPose);
    targetPoseEditor.setTargetPose(targetPose, {
      resetRoll: true,
      markReady: false,
    });
    updateTargetPoseReadout();
  }
  targetPoseEditor.setModeActive(activeControlMode === "xyz");
  targetPoseEditor.setControlsVisible(tcpControlsVisible);
}

function createRobotModel(geometries, ghost = false, d405Geometry = undefined) {
  const root = new THREE.Group();
  root.name = ghost ? "preview-x5" : "actual-x5";
  const origins = [];
  const motions = [];
  const meshes = [];
  let gripperGeometry = null;
  let officialGripperIsDynamic = false;

  const baseMesh = new THREE.Mesh(geometries[0], createLinkMaterial(0, ghost));
  baseMesh.userData.jointIndex = null;
  baseMesh.renderOrder = ghost ? 3 : 1;
  baseMesh.castShadow = !ghost;
  baseMesh.receiveShadow = true;
  root.add(baseMesh);
  meshes.push(baseMesh);

  let parent = root;
  JOINT_SPECS.forEach((spec, index) => {
    const origin = new THREE.Group();
    origin.name = `joint${index + 1}-origin`;
    origin.position.set(...spec.origin);
    origin.rotation.set(...spec.rpy, "XYZ");
    parent.add(origin);
    origins.push(origin);

    const motion = new THREE.Group();
    motion.name = `joint${index + 1}-motion`;
    origin.add(motion);
    motions.push(motion);

    const sourceGeometry = geometries[index + 1];
    const dynamicGeometry = index === 5
      ? createDynamicGripperGeometry(sourceGeometry)
      : { geometry: sourceGeometry, dynamic: false };
    if (index === 5) {
      gripperGeometry = dynamicGeometry.geometry;
      officialGripperIsDynamic = dynamicGeometry.dynamic;
    }
    const mesh = new THREE.Mesh(
      dynamicGeometry.geometry,
      createLinkMaterial(index + 1, ghost),
    );
    mesh.name = `link${index + 1}`;
    mesh.userData.jointIndex = index;
    mesh.renderOrder = ghost ? 3 : 1;
    mesh.castShadow = !ghost;
    mesh.receiveShadow = true;
    motion.add(mesh);
    meshes.push(mesh);
    parent = motion;
  });

  // Official link6.STL contains disconnected finger components. They are
  // translated directly when available; the lightweight overlay is retained
  // only for the offline fallback geometry.
  const gripper = officialGripperIsDynamic ? null : createGripperVisual(ghost);
  if (gripper) motions[5].add(gripper.root);
  // Loading skeletons omit the third argument. Actual and preview robots pass
  // either the official geometry or null to request the dimensional fallback.
  const d405 = d405Geometry === undefined
    ? null
    : createD405Visual(d405Geometry, ghost);
  if (d405) motions[5].add(d405.eefRoot);

  root.traverse((object) => {
    if (!object.isMesh) return;
    object.castShadow = !ghost;
    object.receiveShadow = true;
  });
  root.visible = !ghost;
  scene.add(root);
  return {
    root,
    origins,
    motions,
    meshes,
    appearanceMeshes: [
      ...meshes,
      ...(gripper?.meshes ?? []),
      ...(d405 ? [d405.mesh] : []),
    ],
    gripper,
    gripperGeometry,
    d405,
  };
}

function setModelAngles(model, anglesDeg) {
  model.motions.forEach((motion, index) => {
    const axis = new THREE.Vector3(...JOINT_SPECS[index].axis).normalize();
    motion.quaternion.setFromAxisAngle(axis, THREE.MathUtils.degToRad(anglesDeg[index]));
  });
  model.root.updateMatrixWorld(true);
}

function setModelGripperWidth(model, widthMm) {
  if (!model || !Number.isFinite(widthMm)) return;
  const maximumMm = Number.isFinite(armState.gripper_max_mm)
    ? armState.gripper_max_mm
    : 60;
  const officialUpdated = model.gripperGeometry
    ? setOfficialGripperWidth(model.gripperGeometry, widthMm, maximumMm)
    : false;
  if (!officialUpdated && model.gripper) {
    setGripperVisualWidth(model.gripper, widthMm, maximumMm);
  }
  model.root.updateMatrixWorld(true);
}

function setModelD405Transform(model, transform) {
  const opticalRoot = model?.d405?.opticalRoot;
  if (!opticalRoot) return;
  if (!(transform instanceof THREE.Matrix4)) {
    opticalRoot.visible = false;
    return;
  }
  opticalRoot.matrix.copy(transform);
  opticalRoot.visible = true;
  model.root.updateMatrixWorld(true);
}

function setPreviewAppearance() {
  if (!previewModel) return;
  const limitedTarget = previewKind === "cartesian-limit"
    || previewKind === "cartesian-servo-limit"
    || previewKind === "teleoperation-limit";
  const servoTarget = previewKind === "cartesian-servo"
    || previewKind === "teleoperation";
  const parkTarget = previewKind === "park";
  previewModel.appearanceMeshes.forEach((mesh) => {
    const isJ3 = mesh.userData.jointIndex === J3_LINK_MATERIAL_INDEX - 1;
    mesh.material.color.setHex(
      limitedTarget
        ? LIMIT_PREVIEW_COLOR
        : servoTarget
          ? (isJ3 ? 0x65bfd2 : 0x205665)
          : parkTarget
            ? 0xb994ff
            : (isJ3 ? 0xb9c2c8 : 0x3c4247),
    );
    mesh.material.emissive.setHex(
      limitedTarget
        ? 0x8c1717
        : servoTarget
          ? (isJ3 ? 0x075568 : 0x003440)
          : parkTarget
            ? 0x4f2b7d
            : (isJ3 ? 0x5b6870 : 0x171b1e),
    );
    mesh.material.emissiveIntensity = limitedTarget
      ? 0.56
      : servoTarget
        ? 0.48
        : parkTarget
          ? 0.42
          : 0.34;
    mesh.material.opacity = limitedTarget
      ? 0.42
      : servoTarget
        ? 0.3
        : parkTarget
          ? 0.34
          : 0.36;
  });
}

function createSelectionRing() {
  const group = new THREE.Group();
  group.name = "selected-joint-ring";

  const visible = new THREE.Mesh(
    new THREE.TorusGeometry(0.054, 0.0032, 12, 80),
    new THREE.MeshBasicMaterial({
      color: 0x79e0bf,
      transparent: true,
      opacity: 0.9,
      depthTest: false,
    }),
  );
  visible.renderOrder = 6;
  group.add(visible);

  const hitArea = new THREE.Mesh(
    new THREE.TorusGeometry(0.054, 0.013, 8, 48),
    new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.001, depthWrite: false }),
  );
  hitArea.renderOrder = 7;
  group.add(hitArea);

  const pivot = new THREE.Mesh(
    new THREE.SphereGeometry(0.009, 20, 12),
    new THREE.MeshBasicMaterial({ color: 0x79e0bf, depthTest: false }),
  );
  pivot.renderOrder = 7;
  group.add(pivot);
  ringHitTargets = [hitArea, visible, pivot];
  return group;
}

function createCartesianIndicators() {
  eefDragHandle = new THREE.Mesh(
    new THREE.SphereGeometry(0.015, 24, 16),
    new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.92,
      depthTest: false,
    }),
  );
  eefDragHandle.name = "eef-drag-handle";
  eefDragHandle.position.set(EEF_OFFSET_LINK6_M, 0, 0);
  eefDragHandle.renderOrder = 8;
  eefDragHandle.userData.isEefDragHandle = true;
  actualModel.motions[5].add(eefDragHandle);

  cartesianTargetMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.011, 20, 14),
    new THREE.MeshBasicMaterial({
      color: 0xffb55e,
      transparent: true,
      opacity: 0.9,
      depthTest: false,
    }),
  );
  cartesianTargetMarker.name = "cartesian-target-marker";
  cartesianTargetMarker.renderOrder = 8;
  cartesianTargetMarker.visible = false;
  scene.add(cartesianTargetMarker);

  const guideGeometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(),
    new THREE.Vector3(),
  ]);
  cartesianGuideLine = new THREE.Line(
    guideGeometry,
    new THREE.LineBasicMaterial({
      color: 0xffb55e,
      transparent: true,
      opacity: 0.9,
      depthTest: false,
    }),
  );
  cartesianGuideLine.name = "cartesian-drag-guide";
  cartesianGuideLine.renderOrder = 7;
  cartesianGuideLine.visible = false;
  scene.add(cartesianGuideLine);

  plannedPathLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(),
      new THREE.Vector3(),
    ]),
    new THREE.LineBasicMaterial({
      color: PENDING_PATH_COLOR,
      transparent: true,
      opacity: 0.96,
      depthTest: false,
    }),
  );
  plannedPathLine.name = "planned-motion-path";
  plannedPathLine.renderOrder = 9;
  plannedPathLine.visible = false;
  scene.add(plannedPathLine);

  plannedPathTube = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({
      color: PENDING_PATH_COLOR,
      transparent: true,
      opacity: 0.34,
      depthTest: false,
      depthWrite: false,
    }),
  );
  plannedPathTube.name = "planned-motion-path-tube";
  plannedPathTube.renderOrder = 8;
  plannedPathTube.visible = false;
  scene.add(plannedPathTube);

  executedPathLine = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({
      color: EXECUTED_PATH_COLOR,
      transparent: true,
      opacity: 1,
      depthTest: false,
      depthWrite: false,
    }),
  );
  executedPathLine.name = "executed-motion-path";
  executedPathLine.renderOrder = 10;
  executedPathLine.visible = false;
  executedPathLine.frustumCulled = false;
  scene.add(executedPathLine);

  executedPathTube = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({
      color: EXECUTED_PATH_COLOR,
      transparent: true,
      opacity: 0.58,
      depthTest: false,
      depthWrite: false,
    }),
  );
  executedPathTube.name = "executed-motion-path-tube";
  executedPathTube.renderOrder = 9;
  executedPathTube.visible = false;
  scene.add(executedPathTube);

  plannedPathWaypoints = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({
      color: PENDING_PATH_COLOR,
      size: 0.007,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.82,
      depthTest: false,
      depthWrite: false,
    }),
  );
  plannedPathWaypoints.name = "planned-motion-waypoints";
  plannedPathWaypoints.renderOrder = 10;
  plannedPathWaypoints.visible = false;
  scene.add(plannedPathWaypoints);

  plannedPathFlowLine = new THREE.Line(
    new THREE.BufferGeometry().setAttribute(
      "position",
      new THREE.Float32BufferAttribute(new Float32Array(18 * 3), 3),
    ),
    new THREE.LineBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.96,
      depthTest: false,
      depthWrite: false,
    }),
  );
  plannedPathFlowLine.name = "planned-motion-path-flow";
  plannedPathFlowLine.renderOrder = 11;
  plannedPathFlowLine.visible = false;
  plannedPathFlowLine.frustumCulled = false;
  scene.add(plannedPathFlowLine);
}

function createWoodPBRTextures() {
  const loader = new THREE.TextureLoader();
  const textureRoot = "/textures/oak_veneer_01";
  const load = (filename, colorSpace = THREE.NoColorSpace) => {
    const texture = loader.load(`${textureRoot}/${filename}`);
    texture.colorSpace = colorSpace;
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
    texture.center.set(0.5, 0.5);
    texture.rotation = Math.PI / 2;
    return texture;
  };
  return {
    diffuse: load("oak_veneer_01_diff_2k.jpg", THREE.SRGBColorSpace),
    normal: load("oak_veneer_01_nor_gl_2k.jpg"),
    roughness: load("oak_veneer_01_rough_2k.jpg"),
  };
}

function continuousTableCornerRadius(width, depth) {
  return THREE.MathUtils.clamp(Math.min(width, depth) * 0.075, 0.025, 0.085);
}

function createContinuousTableOutline(width, depth, cornerRadius, segments = 18) {
  const halfWidth = width / 2;
  const halfDepth = depth / 2;
  const radius = Math.min(cornerRadius, halfWidth, halfDepth);
  const exponent = 4;
  const power = 2 / exponent;
  const signedPower = (value) => Math.sign(value) * Math.abs(value) ** power;
  const corners = [
    [halfWidth - radius, halfDepth - radius, Math.PI / 2, 0],
    [halfWidth - radius, -halfDepth + radius, 0, -Math.PI / 2],
    [-halfWidth + radius, -halfDepth + radius, -Math.PI / 2, -Math.PI],
    [-halfWidth + radius, halfDepth - radius, Math.PI, Math.PI / 2],
  ];
  return corners.flatMap(([centerX, centerY, start, end]) => (
    Array.from({ length: segments + 1 }, (_, index) => {
      const angle = THREE.MathUtils.lerp(start, end, index / segments);
      return new THREE.Vector2(
        centerX + radius * signedPower(Math.cos(angle)),
        centerY + radius * signedPower(Math.sin(angle)),
      );
    })
  ));
}

function createContinuousTableGeometry(width, depth, thickness) {
  const edgeRadius = Math.min(
    0.006,
    thickness * 0.22,
    width * 0.01,
    depth * 0.01,
  );
  const innerWidth = Math.max(0.001, width - 2 * edgeRadius);
  const innerDepth = Math.max(0.001, depth - 2 * edgeRadius);
  const cornerRadius = Math.max(
    0.001,
    continuousTableCornerRadius(width, depth) - edgeRadius,
  );
  const shape = new THREE.Shape(
    createContinuousTableOutline(innerWidth, innerDepth, cornerRadius),
  );
  shape.closePath();
  const innerThickness = Math.max(0.001, thickness - 2 * edgeRadius);
  const geometry = new THREE.ExtrudeGeometry(shape, {
    depth: innerThickness,
    steps: 1,
    bevelEnabled: true,
    bevelSegments: 5,
    bevelSize: edgeRadius,
    bevelThickness: edgeRadius,
    curveSegments: 1,
  });
  geometry.translate(0, 0, -innerThickness / 2);
  geometry.computeVertexNormals();
  return geometry;
}

function createCalibrationEnvironment() {
  const root = new THREE.Group();
  root.name = "calibration-environment";
  // Keep the robot base at the existing kinematic origin. The table is moved
  // around it so visual mounting changes never alter IK coordinates.

  const woodTextures = createWoodPBRTextures();
  const tabletopMaterial = new THREE.MeshPhysicalMaterial({
    color: 0xffd4cf,
    map: woodTextures.diffuse,
    normalMap: woodTextures.normal,
    normalScale: new THREE.Vector2(0.32, 0.32),
    roughnessMap: woodTextures.roughness,
    roughness: 0.4,
    metalness: 0,
    clearcoat: 0.28,
    clearcoatRoughness: 0.32,
  });
  const tableSurface = new THREE.Group();
  tableSurface.name = "editable-calibration-tabletop";
  tableSurface.position.set(
    DEFAULT_TABLE_CALIBRATION.xMm / 1000,
    DEFAULT_TABLE_CALIBRATION.yMm / 1000,
    DEFAULT_TABLE_CALIBRATION.zMm / 1000,
  );
  const tabletop = new THREE.Mesh(
    createContinuousTableGeometry(
      DEFAULT_TABLE_CALIBRATION.widthMm / 1000,
      DEFAULT_TABLE_CALIBRATION.depthMm / 1000,
      DEFAULT_TABLE_CALIBRATION.thicknessMm / 1000,
    ),
    tabletopMaterial,
  );
  tabletop.name = "calibration-tabletop";
  tabletop.position.z = -DEFAULT_TABLE_CALIBRATION.thicknessMm / 2000;
  tabletop.userData.calibrationRole = "world-reference-table";
  tableSurface.add(tabletop);
  const tableOutline = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(
      createContinuousTableOutline(
        DEFAULT_TABLE_CALIBRATION.widthMm / 1000,
        DEFAULT_TABLE_CALIBRATION.depthMm / 1000,
        continuousTableCornerRadius(
          DEFAULT_TABLE_CALIBRATION.widthMm / 1000,
          DEFAULT_TABLE_CALIBRATION.depthMm / 1000,
        ),
      ).map((point) => new THREE.Vector3(point.x, point.y, 0.002)),
    ),
    new THREE.LineBasicMaterial({
      color: 0x79e0bf,
      transparent: true,
      opacity: 0.92,
      depthTest: false,
    }),
  );
  tableOutline.name = "calibration-tabletop-outline";
  tableOutline.renderOrder = 18;
  tableOutline.visible = false;
  tableSurface.add(tableOutline);
  root.add(tableSurface);

  root.traverse((object) => {
    if (!object.isMesh) return;
    object.castShadow = true;
    object.receiveShadow = true;
  });
  root.userData.calibrationTable = {
    root: tableSurface,
    mesh: tabletop,
    outline: tableOutline,
    textures: Object.values(woodTextures),
  };
  return root;
}

function normalizeTableCalibration(value = {}) {
  const limits = {
    widthMm: [300, 4000],
    depthMm: [200, 3000],
    thicknessMm: [5, 200],
    xMm: [-3000, 3000],
    yMm: [-3000, 3000],
    zMm: [-1000, 2000],
    yawDeg: [-180, 180],
  };
  return Object.fromEntries(Object.entries(DEFAULT_TABLE_CALIBRATION).map(([key, fallback]) => {
    const requested = Number(value[key]);
    const finite = Number.isFinite(requested) ? requested : fallback;
    const [minimum, maximum] = limits[key];
    return [key, THREE.MathUtils.clamp(finite, minimum, maximum)];
  }));
}

function tableCalibrationEquals(left, right) {
  return Object.keys(DEFAULT_TABLE_CALIBRATION).every((key) => (
    Math.abs(Number(left?.[key]) - Number(right[key])) < 1e-6
  ));
}

function saveTableCalibration(value) {
  try {
    localStorage.setItem(TABLE_CALIBRATION_STORAGE_KEY, JSON.stringify(value));
  } catch (error) {
    console.warn("Unable to save tabletop calibration", error);
  }
}

function loadTableCalibration() {
  try {
    const saved = JSON.parse(localStorage.getItem(TABLE_CALIBRATION_STORAGE_KEY) ?? "null");
    if (saved) return normalizeTableCalibration(saved);
    let legacy = null;
    for (const key of LEGACY_TABLE_CALIBRATION_STORAGE_KEYS) {
      legacy = JSON.parse(localStorage.getItem(key) ?? "null");
      if (legacy) break;
    }
    if (!legacy) return { ...DEFAULT_TABLE_CALIBRATION };
    const calibration = normalizeTableCalibration(legacy);
    // Only migrate the old application defaults.  A manually measured table
    // calibration remains authoritative and must never be moved implicitly.
    const migrated = tableCalibrationEquals(calibration, LEGACY_DEFAULT_TABLE_CALIBRATION)
      || tableCalibrationEquals(calibration, LEGACY_NUDGED_DEFAULT_TABLE_CALIBRATION)
      ? { ...DEFAULT_TABLE_CALIBRATION }
      : calibration;
    saveTableCalibration(migrated);
    return migrated;
  } catch (error) {
    console.warn("Unable to read saved tabletop calibration", error);
    return { ...DEFAULT_TABLE_CALIBRATION };
  }
}

function syncTableCalibrationInputs() {
  Object.entries(elements.tableInputs).forEach(([key, input]) => {
    if (!input) return;
    const precision = ["xMm", "yMm", "zMm", "yawDeg"].includes(key) ? 1 : 0;
    input.value = tableCalibrationState[key].toFixed(precision);
  });
}

function applyTableCalibration(value, { persist = true, syncInputs = false } = {}) {
  tableCalibrationState = normalizeTableCalibration(value);
  if (!calibrationTable) return;

  const width = tableCalibrationState.widthMm / 1000;
  const depth = tableCalibrationState.depthMm / 1000;
  const thickness = tableCalibrationState.thicknessMm / 1000;
  calibrationTable.mesh.geometry.dispose();
  calibrationTable.mesh.geometry = createContinuousTableGeometry(width, depth, thickness);
  calibrationTable.mesh.position.z = -thickness / 2;
  calibrationTable.root.position.set(
    tableCalibrationState.xMm / 1000 - calibrationEnvironment.position.x,
    tableCalibrationState.yMm / 1000 - calibrationEnvironment.position.y,
    tableCalibrationState.zMm / 1000 - calibrationEnvironment.position.z,
  );
  calibrationTable.root.rotation.z = THREE.MathUtils.degToRad(tableCalibrationState.yawDeg);

  calibrationTable.outline.geometry.dispose();
  calibrationTable.outline.geometry = new THREE.BufferGeometry().setFromPoints(
    createContinuousTableOutline(
      width,
      depth,
      continuousTableCornerRadius(width, depth),
    ).map((point) => new THREE.Vector3(point.x, point.y, 0.002)),
  );
  calibrationTable.textures?.forEach((texture) => {
    texture.repeat.set(width / 1.83, depth / 1.83);
    texture.needsUpdate = true;
  });
  calibrationTable.root.updateMatrixWorld(true);
  if (syncInputs) syncTableCalibrationInputs();
  if (persist) saveTableCalibration(tableCalibrationState);
  window.dispatchEvent(new CustomEvent("arx5-scene-calibration"));
}

window.arx5ApplyTableCalibration = (calibration) => {
  if (!calibration || ![calibration.xMm, calibration.yMm, calibration.yawDeg]
    .every((value) => Number.isFinite(Number(value)))) return false;
  applyTableCalibration({
    ...tableCalibrationState,
    ...calibration,
  }, { syncInputs: true });
  return true;
};

function setTableCalibrationEditing(active) {
  tableCalibrationEditing = Boolean(active);
  if (elements.tableCalibrationPanel) {
    elements.tableCalibrationPanel.hidden = !tableCalibrationEditing;
  }
  if (elements.tableCalibrationButton) {
    elements.tableCalibrationButton.classList.toggle("selected", tableCalibrationEditing);
    elements.tableCalibrationButton.setAttribute("aria-expanded", String(tableCalibrationEditing));
  }
  if (calibrationTable?.outline) calibrationTable.outline.visible = tableCalibrationEditing;
  if (tableCalibrationEditing) syncTableCalibrationInputs();
}

function setTableAutoAlignStatus(message, state = "idle") {
  if (!elements.tableAutoAlignStatus) return;
  elements.tableAutoAlignStatus.textContent = message;
  elements.tableAutoAlignStatus.dataset.state = state;
}

function normalizeLightCalibration(value = {}) {
  const limits = {
    xMm: [-4000, 4000],
    yMm: [-4000, 4000],
    zMm: [100, 5000],
    intensity: [0.2, 8],
  };
  return Object.fromEntries(Object.entries(DEFAULT_LIGHT_CALIBRATION).map(([key, fallback]) => {
    const requested = Number(value[key]);
    const finite = Number.isFinite(requested) ? requested : fallback;
    const [minimum, maximum] = limits[key];
    return [key, THREE.MathUtils.clamp(finite, minimum, maximum)];
  }));
}

function loadLightCalibration() {
  try {
    const saved = JSON.parse(localStorage.getItem(LIGHT_CALIBRATION_STORAGE_KEY) ?? "null");
    return normalizeLightCalibration(saved ?? DEFAULT_LIGHT_CALIBRATION);
  } catch (error) {
    console.warn("Unable to read saved light calibration", error);
    return { ...DEFAULT_LIGHT_CALIBRATION };
  }
}

function syncLightCalibrationInputs() {
  Object.entries(elements.lightInputs).forEach(([key, input]) => {
    if (!input) return;
    input.value = lightCalibrationState[key].toFixed(key === "intensity" ? 1 : 0);
  });
}

function applyLightCalibration(value, { persist = true, syncInputs = false } = {}) {
  lightCalibrationState = normalizeLightCalibration(value);
  keyLight.position.set(
    lightCalibrationState.xMm / 1000,
    lightCalibrationState.yMm / 1000,
    lightCalibrationState.zMm / 1000,
  );
  keyLight.intensity = lightCalibrationState.intensity;
  keyLight.updateMatrixWorld(true);
  keyLightHelper.update();
  if (syncInputs) syncLightCalibrationInputs();
  if (persist) {
    try {
      localStorage.setItem(LIGHT_CALIBRATION_STORAGE_KEY, JSON.stringify(lightCalibrationState));
    } catch (error) {
      console.warn("Unable to save light calibration", error);
    }
  }
}

function setLightCalibrationEditing(active) {
  lightCalibrationEditing = Boolean(active);
  if (elements.lightCalibrationPanel) {
    elements.lightCalibrationPanel.hidden = !lightCalibrationEditing;
  }
  if (elements.lightCalibrationButton) {
    elements.lightCalibrationButton.classList.toggle("selected", lightCalibrationEditing);
    elements.lightCalibrationButton.setAttribute("aria-expanded", String(lightCalibrationEditing));
  }
  keyLightHelper.visible = lightCalibrationEditing;
  if (lightCalibrationEditing) syncLightCalibrationInputs();
}

function focusTabletop() {
  cameraCalibrationViewActive = false;
  restoreSceneCameraProjection();
  calibrationCamera.helper.visible = true;
  const center = new THREE.Vector3(
    tableCalibrationState.xMm / 1000,
    tableCalibrationState.yMm / 1000,
    tableCalibrationState.zMm / 1000,
  );
  const span = Math.max(
    tableCalibrationState.widthMm,
    tableCalibrationState.depthMm,
  ) / 1000;
  const offset = new THREE.Vector3(0, -1.05, 0.72)
    .applyAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(tableCalibrationState.yawDeg))
    .setLength(THREE.MathUtils.clamp(span * 0.95, 0.75, 2.1));
  controls.target.copy(center);
  camera.position.copy(center).add(offset);
  controls.update();
}

function createCalibrationCamera() {
  const opticalCamera = new THREE.PerspectiveCamera(
    D455_RGB_VERTICAL_FOV_DEG,
    4 / 3,
    0.06,
    0.95,
  );
  opticalCamera.name = "d455-calibration-camera";
  opticalCamera.up.set(0, 0, 1);
  opticalCamera.userData.calibrationRole = "external-camera";
  opticalCamera.userData.frame = "camera-optical";
  opticalCamera.userData.model = "Intel RealSense D455 RGB";

  const housingMaterial = new THREE.MeshPhysicalMaterial({
    color: 0x202428,
    roughness: 0.34,
    metalness: 0.5,
    clearcoat: 0.25,
    clearcoatRoughness: 0.3,
  });
  const body = new THREE.Mesh(
    new RoundedBoxGeometry(0.124, 0.035, 0.032, 3, 0.005),
    housingMaterial,
  );
  body.name = "calibration-camera-body";
  body.position.z = 0.022;
  body.castShadow = true;
  opticalCamera.add(body);

  const lens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.012, 0.012, 0.006, 28),
    new THREE.MeshStandardMaterial({
      color: 0x172a31,
      emissive: 0x0b4050,
      emissiveIntensity: 0.5,
      roughness: 0.22,
      metalness: 0.5,
    }),
  );
  lens.name = "calibration-camera-lens";
  lens.rotation.x = Math.PI / 2;
  lens.position.z = 0.003;
  opticalCamera.add(lens);

  const opticalAxes = new THREE.Group();
  opticalAxes.name = "camera-optical-axes";
  opticalAxes.add(
    new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.12, 0xff665f, 0.025, 0.012),
    new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), new THREE.Vector3(), 0.12, 0x70e89f, 0.025, 0.012),
    new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.18, 0x74a9ff, 0.03, 0.014),
  );
  opticalAxes.traverse((object) => {
    if (!object.material) return;
    object.material.transparent = true;
    object.material.opacity = 0.86;
    object.material.depthTest = false;
  });
  opticalAxes.visible = false;
  opticalCamera.add(opticalAxes);

  opticalCamera.updateProjectionMatrix();
  const helper = new THREE.CameraHelper(opticalCamera);
  helper.name = "d455-calibration-frustum";
  helper.renderOrder = 15;
  helper.material.transparent = true;
  helper.material.opacity = 0.28;
  helper.material.depthTest = false;
  helper.setColors(
    new THREE.Color(0x8b959b),
    new THREE.Color(0x79e0bf),
    new THREE.Color(0xffffff),
    new THREE.Color(0x74a9ff),
    new THREE.Color(0x8b959b),
  );
  return { opticalCamera, helper, opticalAxes };
}

function normalizeCameraCalibration(value = {}) {
  const limits = {
    xMm: [-3000, 3000],
    yMm: [-3000, 3000],
    zMm: [-1000, 3000],
    rollDeg: [-180, 180],
    pitchDeg: [-89, 89],
    yawDeg: [-180, 180],
  };
  return Object.fromEntries(Object.entries(DEFAULT_CAMERA_CALIBRATION).map(([key, fallback]) => {
    const requested = Number(value[key]);
    const finite = Number.isFinite(requested) ? requested : fallback;
    const [minimum, maximum] = limits[key];
    return [key, THREE.MathUtils.clamp(finite, minimum, maximum)];
  }));
}

function loadCameraCalibration() {
  try {
    const saved = JSON.parse(localStorage.getItem(CAMERA_CALIBRATION_STORAGE_KEY) ?? "null");
    if (saved) return normalizeCameraCalibration(saved);
    const legacy = JSON.parse(
      localStorage.getItem(LEGACY_CAMERA_CALIBRATION_STORAGE_KEY) ?? "null",
    );
    const legacyWasDefault = legacy
      && Number(legacy.xMm) === 0
      && Number(legacy.yMm) === -850
      && Number(legacy.zMm) === 205
      && Number(legacy.rollDeg) === 0
      && Number(legacy.pitchDeg) === 0
      && Number(legacy.yawDeg) === 0;
    return normalizeCameraCalibration(
      !legacy || legacyWasDefault ? DEFAULT_CAMERA_CALIBRATION : legacy,
    );
  } catch (error) {
    console.warn("Unable to read saved camera calibration", error);
    return { ...DEFAULT_CAMERA_CALIBRATION };
  }
}

function syncCameraCalibrationInputs() {
  Object.entries(elements.cameraInputs).forEach(([key, input]) => {
    if (!input) return;
    input.value = cameraCalibrationState[key].toFixed(1);
  });
}

function applyCameraCalibration(value, { persist = true, syncInputs = false } = {}) {
  cameraCalibrationState = normalizeCameraCalibration(value);
  if (!calibrationCamera) return;

  const position = new THREE.Vector3(
    cameraCalibrationState.xMm / 1000,
    cameraCalibrationState.yMm / 1000,
    cameraCalibrationState.zMm / 1000,
  );
  const worldUp = new THREE.Vector3(0, 0, 1);
  const forward = new THREE.Vector3(0, 1, 0).applyAxisAngle(
    worldUp,
    THREE.MathUtils.degToRad(cameraCalibrationState.yawDeg),
  );
  const right = forward.clone().cross(worldUp).normalize();
  const pitch = THREE.MathUtils.degToRad(cameraCalibrationState.pitchDeg);
  const imageUp = worldUp.clone().applyAxisAngle(right, pitch);
  forward.applyAxisAngle(right, pitch);
  imageUp.applyAxisAngle(forward, THREE.MathUtils.degToRad(cameraCalibrationState.rollDeg));

  calibrationCamera.opticalCamera.position.copy(position);
  calibrationCamera.opticalCamera.up.copy(imageUp);
  calibrationCamera.opticalCamera.lookAt(position.clone().add(forward));
  calibrationCamera.opticalCamera.updateMatrixWorld(true);
  calibrationCamera.helper.update();
  if (syncInputs) syncCameraCalibrationInputs();
  if (persist) {
    try {
      localStorage.setItem(CAMERA_CALIBRATION_STORAGE_KEY, JSON.stringify(cameraCalibrationState));
    } catch (error) {
      console.warn("Unable to save camera calibration", error);
    }
  }
  if (cameraCalibrationViewActive) viewFromCalibrationCamera();
}

function applyHandeyeCameraTransform(value) {
  if (!Array.isArray(value) || value.length !== 4
      || !value.every((row) => Array.isArray(row) && row.length === 4
        && row.every((component) => Number.isFinite(Number(component))))) {
    return;
  }
  const matrix = value.map((row) => row.map(Number));
  if (matrix[3][0] !== 0 || matrix[3][1] !== 0
      || matrix[3][2] !== 0 || matrix[3][3] !== 1) return;
  const position = new THREE.Vector3(matrix[0][3], matrix[1][3], matrix[2][3]);
  const forward = new THREE.Vector3(matrix[0][2], matrix[1][2], matrix[2][2]).normalize();
  const imageUp = new THREE.Vector3(-matrix[0][1], -matrix[1][1], -matrix[2][1]).normalize();
  if (!Number.isFinite(forward.length()) || forward.lengthSq() < 0.5) return;
  calibrationCamera.opticalCamera.position.copy(position);
  calibrationCamera.opticalCamera.up.copy(imageUp);
  calibrationCamera.opticalCamera.lookAt(position.clone().add(forward));
  calibrationCamera.opticalCamera.updateMatrixWorld(true);
  calibrationCamera.helper.update();
  handeyeCameraTransform = matrix;
}

function applyHandeyeD405Transform(value) {
  const matrix = matrixFromRowMajorTransform(value);
  const validHomogeneousRow = matrix
    && Number(value[3][0]) === 0
    && Number(value[3][1]) === 0
    && Number(value[3][2]) === 0
    && Number(value[3][3]) === 1;
  handeyeD405Transform = validHomogeneousRow ? matrix : null;
  setModelD405Transform(actualModel, handeyeD405Transform);
  setModelD405Transform(previewModel, handeyeD405Transform);
}

function setCameraCalibrationEditing(active) {
  cameraCalibrationEditing = Boolean(active);
  if (elements.cameraCalibrationPanel) {
    elements.cameraCalibrationPanel.hidden = !cameraCalibrationEditing;
  }
  if (elements.cameraCalibrationButton) {
    elements.cameraCalibrationButton.classList.toggle("selected", cameraCalibrationEditing);
    elements.cameraCalibrationButton.setAttribute("aria-expanded", String(cameraCalibrationEditing));
  }
  calibrationCamera.helper.visible = true;
  calibrationCamera.helper.material.opacity = cameraCalibrationEditing ? 0.72 : 0.28;
  calibrationCamera.opticalAxes.visible = cameraCalibrationEditing;
  if (cameraCalibrationEditing) syncCameraCalibrationInputs();
}

function restoreSceneCameraProjection() {
  camera.fov = DEFAULT_SCENE_CAMERA_FOV_DEG;
  camera.up.set(0, 0, 1);
  camera.updateProjectionMatrix();
}

function focusCalibrationCamera() {
  cameraCalibrationViewActive = false;
  restoreSceneCameraProjection();
  calibrationCamera.helper.visible = true;
  const center = calibrationCamera.opticalCamera.position;
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(0.42, -0.42, 0.28));
  controls.update();
}

function viewFromCalibrationCamera() {
  cameraCalibrationViewActive = true;
  const source = calibrationCamera.opticalCamera;
  source.updateMatrixWorld(true);
  const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(source.quaternion);
  const imageUp = new THREE.Vector3(0, 1, 0).applyQuaternion(source.quaternion);
  camera.position.copy(source.position);
  camera.up.copy(imageUp);
  camera.fov = D455_RGB_VERTICAL_FOV_DEG;
  camera.updateProjectionMatrix();
  controls.target.copy(source.position).addScaledVector(forward, 0.9);
  calibrationCamera.helper.visible = false;
  controls.update();
}

function setCartesianGuide(start, target, color = null, showLine = true) {
  if (!cartesianTargetMarker || !cartesianGuideLine) return;
  if (!start || !target) {
    cartesianTargetMarker.visible = false;
    cartesianGuideLine.visible = false;
    return;
  }
  const guideColor = color ?? 0xffb55e;
  cartesianTargetMarker.material.color.setHex(guideColor);
  cartesianGuideLine.material.color.setHex(guideColor);
  cartesianTargetMarker.position.copy(target);
  const positions = cartesianGuideLine.geometry.attributes.position;
  positions.setXYZ(0, start.x, start.y, start.z);
  positions.setXYZ(1, target.x, target.y, target.z);
  positions.needsUpdate = true;
  cartesianGuideLine.geometry.computeBoundingSphere();
  cartesianTargetMarker.visible = true;
  cartesianGuideLine.visible = showLine;
}

function setPlannedPath(pathMm, pathMode = "smooth") {
  if (!plannedPathLine) return;
  const valid = Array.isArray(pathMm)
    && pathMm.length >= 2
    && pathMm.every((point) => (
      Array.isArray(point)
      && point.length === 3
      && point.every(Number.isFinite)
    ));
  if (!valid) {
    plannedPathLine.visible = false;
    if (plannedPathTube) plannedPathTube.visible = false;
    if (executedPathLine) executedPathLine.visible = false;
    if (executedPathTube) executedPathTube.visible = false;
    if (plannedPathWaypoints) plannedPathWaypoints.visible = false;
    if (plannedPathFlowLine) plannedPathFlowLine.visible = false;
    plannedPathCurve = null;
    plannedPathFlowCurve = null;
    plannedPathSamples = [];
    executedPathFeedbackPoints = [];
    executedPathGeometryDirty = false;
    plannedPathExecutionActive = false;
    plannedPathExecutionProgress = 0;
    plannedPathFeedbackDirty = false;
    renderedPathProgress = -1;
    plannedPathFlowStartProgress = 0;
    plannedPathMode = null;
    return;
  }
  const points = pathMm.map((point) => new THREE.Vector3(
    point[0] / 1000.0,
    point[1] / 1000.0,
    point[2] / 1000.0,
  ));
  const current = currentEefWorldPosition();
  if (current && current.toArray().every(Number.isFinite)
      && current.distanceTo(points[0]) > 0.001) {
    points.unshift(current);
  }
  plannedPathMode = pathMode;
  const curve = new THREE.CurvePath();
  for (let index = 1; index < points.length; index += 1) {
    if (points[index - 1].distanceToSquared(points[index]) > 1e-12) {
      curve.add(new THREE.LineCurve3(points[index - 1], points[index]));
    }
  }
  if (curve.curves.length === 0) {
    if (plannedPathFlowLine) plannedPathFlowLine.visible = false;
    return;
  }
  curve.updateArcLengths();
  plannedPathCurve = curve;
  const sampleCount = Math.max(80, Math.min(360, points.length * 4));
  plannedPathSamples = curve.getSpacedPoints(sampleCount);
  executedPathFeedbackPoints = [];
  executedPathGeometryDirty = false;
  executedPathLine.visible = false;
  executedPathTube.visible = false;
  plannedPathExecutionActive = false;
  plannedPathExecutionProgress = 0;
  plannedPathFeedbackDirty = false;
  renderedPathProgress = -1;
  plannedPathFlowStartProgress = 0;
  updatePlannedPathSplit(true);
}

function pathCurveFromPoints(points) {
  if (!Array.isArray(points) || points.length < 2) return null;
  const curve = new THREE.CurvePath();
  for (let index = 1; index < points.length; index += 1) {
    if (points[index - 1].distanceToSquared(points[index]) > 1e-12) {
      curve.add(new THREE.LineCurve3(points[index - 1], points[index]));
    }
  }
  if (curve.curves.length === 0) return null;
  curve.updateArcLengths();
  return curve;
}

function sampledPathRange(startProgress, endProgress) {
  if (!plannedPathCurve || endProgress - startProgress < 1e-6) return [];
  const totalSegments = Math.max(1, plannedPathSamples.length - 1);
  const steps = Math.max(1, Math.ceil((endProgress - startProgress) * totalSegments));
  return Array.from({ length: steps + 1 }, (_unused, index) => (
    plannedPathCurve.getPointAt(
      startProgress + (endProgress - startProgress) * index / steps,
    )
  ));
}

function replacePathGeometry(line, tube, points, radius, visible, buildTube = true) {
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points);
  line.visible = visible && points.length >= 2;
  const curve = pathCurveFromPoints(points);
  if (!buildTube) {
    tube.visible = false;
    return curve;
  }
  tube.geometry.dispose();
  if (!curve || !visible) {
    tube.geometry = new THREE.BufferGeometry();
    tube.visible = false;
    return null;
  }
  tube.geometry = new THREE.TubeGeometry(
    curve,
    Math.max(12, Math.min(240, points.length * 2)),
    radius,
    8,
    false,
  );
  tube.visible = true;
  return curve;
}

function updatePlannedPathSplit(force = false, now = performance.now()) {
  if (!plannedPathCurve) return;
  const progress = THREE.MathUtils.clamp(plannedPathExecutionProgress, 0, 1);
  if (!force && Math.abs(progress - renderedPathProgress) < 0.006) return;
  if (!force
      && now - lastPathSplitGeometryUpdateAt < PATH_GEOMETRY_UPDATE_INTERVAL_MS) return;
  renderedPathProgress = progress;
  lastPathSplitGeometryUpdateAt = now;
  if (plannedPathExecutionActive) {
    const position = plannedPathLine.geometry.getAttribute("position");
    const pointCount = position?.count ?? 0;
    const firstPoint = pointCount > 1
      ? Math.min(pointCount - 1, Math.floor(progress * (pointCount - 1)))
      : 0;
    plannedPathLine.geometry.setDrawRange(
      firstPoint,
      Math.max(0, pointCount - firstPoint),
    );
    plannedPathLine.visible = progress < 0.999 && pointCount - firstPoint >= 2;
    plannedPathTube.visible = false;
    plannedPathWaypoints.visible = false;
    plannedPathFlowCurve = plannedPathCurve;
    plannedPathFlowStartProgress = progress;
    plannedPathFlowLine.visible = progress < 0.999;
    return;
  }
  const pendingPoints = sampledPathRange(progress, 1);
  const radius = plannedPathMode === "cube-pick" ? 0.004 : 0.0032;
  const buildTube = true;
  plannedPathFlowCurve = replacePathGeometry(
    plannedPathLine,
    plannedPathTube,
    pendingPoints,
    radius,
    progress < 0.999,
    buildTube,
  );
  const waypointStride = Math.max(1, Math.ceil(pendingPoints.length / 18));
  const waypointPoints = pendingPoints.filter((_point, index) => (
    index === 0 || index === pendingPoints.length - 1 || index % waypointStride === 0
  ));
  plannedPathWaypoints.geometry.dispose();
  plannedPathWaypoints.geometry = new THREE.BufferGeometry().setFromPoints(waypointPoints);
  plannedPathWaypoints.visible = progress < 0.999 && waypointPoints.length > 0;
  plannedPathFlowLine.visible = progress < 0.999 && Boolean(plannedPathFlowCurve);
  plannedPathFlowStartProgress = 0;
}

function feedbackEefPosition() {
  if (!finitePose(armState.eef_pose_6d)) return null;
  const point = new THREE.Vector3(
    Number(armState.eef_pose_6d[0]),
    Number(armState.eef_pose_6d[1]),
    Number(armState.eef_pose_6d[2]),
  );
  return point.toArray().every(Number.isFinite) ? point : null;
}

function decimateExecutedPathFeedback() {
  while (executedPathFeedbackPoints.length > MAX_EXECUTED_PATH_POINTS) {
    const latest = executedPathFeedbackPoints.at(-1);
    executedPathFeedbackPoints = executedPathFeedbackPoints.filter(
      (_point, index) => index === 0
        || index === executedPathFeedbackPoints.length - 1
        || index % 2 === 0,
    );
    if (executedPathFeedbackPoints.at(-1) !== latest) {
      executedPathFeedbackPoints.push(latest);
    }
  }
}

function syncExecutedPathLineGeometry() {
  let position = executedPathLine.geometry.getAttribute("position");
  if (!position || position.count !== MAX_EXECUTED_PATH_POINTS) {
    executedPathLine.geometry.dispose();
    const geometry = new THREE.BufferGeometry();
    position = new THREE.Float32BufferAttribute(
      new Float32Array(MAX_EXECUTED_PATH_POINTS * 3),
      3,
    );
    position.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute("position", position);
    geometry.setDrawRange(0, 0);
    executedPathLine.geometry = geometry;
  }
  executedPathFeedbackPoints.forEach((point, index) => {
    position.setXYZ(index, point.x, point.y, point.z);
  });
  position.needsUpdate = true;
  executedPathLine.geometry.setDrawRange(0, executedPathFeedbackPoints.length);
  executedPathLine.visible = executedPathFeedbackPoints.length >= 2;
  executedPathTube.visible = false;
}

function updateExecutedFeedbackPath(force = false, now = performance.now()) {
  const point = feedbackEefPosition();
  let changed = false;
  if (point) {
    const previous = executedPathFeedbackPoints.at(-1);
    if (!previous || previous.distanceToSquared(point) >= 0.00000016) {
      executedPathFeedbackPoints.push(point);
      changed = true;
    } else if (force && previous) {
      executedPathFeedbackPoints[executedPathFeedbackPoints.length - 1] = point;
      changed = true;
    }
  }
  if (changed) {
    decimateExecutedPathFeedback();
    executedPathGeometryDirty = true;
  }
  if (!force
      && now - lastExecutedPathGeometryUpdateAt < PATH_GEOMETRY_UPDATE_INTERVAL_MS) return;
  if (!executedPathGeometryDirty && !force) return;
  if (plannedPathExecutionActive) {
    syncExecutedPathLineGeometry();
    executedPathGeometryDirty = false;
    lastExecutedPathGeometryUpdateAt = now;
    return;
  }
  const radius = plannedPathMode === "cube-pick" ? 0.004 : 0.0032;
  replacePathGeometry(
    executedPathLine,
    executedPathTube,
    executedPathFeedbackPoints,
    radius,
    executedPathFeedbackPoints.length >= 2,
    true,
  );
  executedPathGeometryDirty = false;
  lastExecutedPathGeometryUpdateAt = now;
}

function beginPlannedPathExecution() {
  if (!plannedPathCurve) return;
  plannedPathExecutionActive = true;
  plannedPathExecutionProgress = 0;
  plannedPathFeedbackDirty = true;
  executedPathFeedbackPoints = [];
  executedPathGeometryDirty = true;
  updateExecutedFeedbackPath(true);
  updatePlannedPathSplit(true);
}

function nearestPlannedPathProgress(point) {
  if (!point || plannedPathSamples.length < 2) return plannedPathExecutionProgress;
  let nearestIndex = 0;
  let nearestDistance = Infinity;
  plannedPathSamples.forEach((sample, index) => {
    const distance = sample.distanceToSquared(point);
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearestIndex = index;
    }
  });
  return nearestIndex / (plannedPathSamples.length - 1);
}

function finishPlannedPathExecution(completed) {
  if (!plannedPathCurve) return;
  if (completed) plannedPathExecutionProgress = 1;
  plannedPathExecutionActive = false;
  plannedPathFeedbackDirty = false;
  updateExecutedFeedbackPath(true);
  updatePlannedPathSplit(true);
}

function setEefHandlePlane(plane = null) {
  if (!eefDragHandle) return;
  eefDragHandle.material.color.setHex(plane?.color ?? 0xffffff);
  eefDragHandle.scale.setScalar(plane ? 1.18 : 1.0);
}

function attachSelectionRing() {
  if (!actualModel || !selectionRing) return;
  selectionRing.removeFromParent();
  selectionRing.position.set(0, 0, 0);
  selectionRing.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(...JOINT_SPECS[selectedJoint].axis).normalize(),
  );
  actualModel.origins[selectedJoint].add(selectionRing);
  actualModel.root.updateMatrixWorld(true);
}

function updateModelHighlight() {
  if (!actualModel) return;
  actualModel.meshes.forEach((mesh, meshIndex) => {
    const selected = activeControlMode === "joint"
      && meshIndex === selectedJoint + 1;
    mesh.material.emissive.setHex(selected ? 0x0b6f57 : 0x070807);
    mesh.material.emissiveIntensity = selected ? 0.34 : 0.08;
  });
  attachSelectionRing();
}

function updatePreviewVisibility() {
  if (!previewModel) return;
  setPreviewAppearance();
  const visible = previewDirty
    && (!hasTeleoperationPreview() || activeControlMode === "hand");
  previewModel.root.visible = visible;
  previewModel.meshes.forEach((mesh, meshIndex) => {
    mesh.visible = visible && (
      hasFullRobotPreview() || meshIndex >= selectedJoint + 1
    );
  });
}

function resetPreview({ quiet = false, preservePath = false } = {}) {
  const clearedCartesianPreview = hasCartesianVisualPreview();
  const clearedParkPreview = previewKind === "park";
  previewAngles = actualAngles.slice();
  previewDirty = false;
  previewKind = null;
  cartesianPreview = null;
  commandPending = false;
  formalPreviewFallbackDelta = null;
  formalPreviewDragGeneration = null;
  formalPreviewDragSource = null;
  formalPreviewTargetSignature = null;
  livePreviewQueued = false;
  if (livePreviewTimer) {
    clearTimeout(livePreviewTimer);
    livePreviewTimer = undefined;
  }
  displayedPreviewAngles = previewAngles.slice();
  if (!preservePath) setPlannedPath(null);
  if (!cartesianDrag) setCartesianGuide(null, null);
  if (previewModel) setModelAngles(previewModel, previewAngles);
  updatePreviewVisibility();
  updateInspector();
  updateCartesianInspector();
  if (clearedCartesianPreview && targetPoseEditor) {
    targetPoseEditor.setValidationState("pending");
    setTargetGhostAppearance("pending");
  }
  if (clearedParkPreview) cancelParkPreview(false);
  if (!quiet && armState.connected) {
    setPreviewStatus("预览已恢复为当前反馈姿态。", "");
  }
}

function selectJoint(index) {
  if (!Number.isInteger(index) || index < 0 || index >= JOINT_SPECS.length) return;
  if (index !== selectedJoint && previewDirty) resetPreview({ quiet: true });
  selectedJoint = index;
  elements.jointButtons.forEach((button, buttonIndex) => {
    button.classList.toggle("selected", buttonIndex === selectedJoint);
    button.setAttribute("aria-pressed", buttonIndex === selectedJoint ? "true" : "false");
  });
  elements.dialLabel.textContent = `J${selectedJoint + 1}`;
  updateModelHighlight();
  updateInspector();
}

function setPreviewStatus(text, className = "") {
  elements.previewStatus.textContent = text;
  elements.previewStatus.className = `robot-preview-status ${className}`.trim();
}

function setXyzStatus(text, className = "") {
  elements.xyzStatus.textContent = text;
  elements.xyzStatus.className = `xyz-status ${className}`.trim();
}

function setShortcutStatus(text, className = "") {
  if (activeControlMode === "xyz") {
    setXyzStatus(text, className);
  } else {
    setPreviewStatus(text, className);
  }
}

function gripperKeyParams(direction) {
  const currentWidth = Number(armState.gripper_position_mm);
  const maximumWidth = Number(armState.gripper_max_mm);
  if (!motionAvailable() || !Number.isFinite(currentWidth)
      || !Number.isFinite(maximumWidth)) {
    setShortcutStatus("当前状态不能使用夹爪快捷键。", "error");
    return null;
  }
  const torqueNm = elements.gripperTorque?.valueAsNumber;
  const minimumMm = elements.gripperMinimum?.valueAsNumber;
  if (!Number.isFinite(torqueNm) || !Number.isFinite(minimumMm)) {
    setShortcutStatus("请先填写有效的夹爪扭矩和最低开度。", "error");
    return null;
  }
  if ((direction < 0 && currentWidth <= minimumMm + 0.1)
      || (direction > 0 && currentWidth >= maximumWidth - 0.1)) {
    setShortcutStatus(
      direction < 0 ? "夹爪已达到最低开度。" : "夹爪已达到最大开度。",
      "",
    );
    return null;
  }
  return { direction, torque_nm: torqueNm, minimum_mm: minimumMm };
}

function updateGripperKeyDirection(direction) {
  if (direction === gripperServoDirection) return true;
  if (gripperServoDirection === 0) {
    const params = gripperKeyParams(direction);
    if (!params || sendGripperServoMessage("start", params) === null) return false;
    gripperKeyHeartbeatTimer = window.setInterval(() => {
      if (gripperServoDirection !== 0) {
        sendGripperServoMessage("update", { direction: gripperServoDirection });
      }
    }, GRIPPER_KEY_HEARTBEAT_INTERVAL_MS);
  } else if (sendGripperServoMessage("update", { direction }) === null) {
    return false;
  }
  gripperServoDirection = direction;
  setShortcutStatus(
    direction > 0 ? "按住 D：夹爪持续张开，松开停止。" : "按住 F：夹爪持续闭合，松开停止。",
    "ready",
  );
  return true;
}

function stopGripperKeyControl({ notifyServer = true } = {}) {
  if (gripperKeyHeartbeatTimer) window.clearInterval(gripperKeyHeartbeatTimer);
  gripperKeyHeartbeatTimer = undefined;
  gripperKeysHeld.clear();
  const wasActive = gripperServoDirection !== 0;
  gripperServoDirection = 0;
  if (wasActive && notifyServer) sendGripperServoMessage("stop");
}

function updateCartesianInspector() {
  const currentMm = finitePose(armState.eef_pose_6d)
    ? armState.eef_pose_6d.slice(0, 3).map((value) => Number(value) * 1000)
    : null;
  const available = modelsReady && motionAvailable() && !parkControlFollowing;
  const deltaValid = validCartesianDelta();
  const targetEditing = Boolean(targetPoseEditor?.drag)
    || [TARGET_EDIT_STATE.EDIT_POSITION, TARGET_EDIT_STATE.EDIT_ORIENTATION]
      .includes(targetPoseEditor?.state);

  targetPoseEditor?.setInteractionEnabled(
    !armState.busy && !commandPending && !parkControlFollowing,
  );

  elements.xyzCurrent.textContent = formatXyz(currentMm);
  elements.controlModeButtons.forEach((button) => {
    button.disabled = Boolean(
      armState.busy || commandPending || parkControlFollowing
      || targetPoseEditor?.drag || ringDrag,
    );
  });
  Object.values(elements.xyzInputs).forEach((input) => {
    input.disabled = !available;
  });
  elements.targetRoll.disabled = !available;
  elements.speedInput.disabled = Boolean(
    armState.busy || armState.servo_active || commandPending || parkControlFollowing
    || targetPoseEditor?.drag || ringDrag,
  );
  const servoAvailable = modelsReady
    && armState.connected
    && ["active", "damping"].includes(armState.mode)
    && !armState.busy
    && !commandPending
    && !parkControlFollowing;
  if (!servoAvailable) {
    elements.servoEnabled.disabled = true;
  } else {
    elements.servoEnabled.disabled = false;
  }
  elements.pathModeButtons.forEach((button) => {
    button.disabled = Boolean(
      armState.busy || armState.servo_active || commandPending || parkControlFollowing
      || targetPoseEditor?.drag || ringDrag,
    );
  });
  elements.xyzQuickButtons.forEach((button) => { button.disabled = !available; });
  elements.xyzClearButton.disabled = !available;
  elements.xyzPreviewButton.disabled = !available || !deltaValid || commandPending || targetEditing;
  elements.xyzExecuteButton.disabled = !available
    || previewKind !== "cartesian"
    || !cartesianPreview
    || commandPending
    || targetEditing;
  if (eefDragHandle) {
    eefDragHandle.visible = modelsReady && activeControlMode === "xyz"
      && tcpControlsVisible
      && elements.servoEnabled.checked;
  }

  if (!armState.connected) {
    setXyzStatus("", "");
  } else if (armState.mode === "fault") {
    setXyzStatus("当前为故障阻尼，不能规划 XYZ 运动。", "error");
  } else if (armState.busy || commandPending) {
    setXyzStatus("正在计算或执行，请等待反馈。", "");
  } else if (targetPoseEditor?.state === TARGET_EDIT_STATE.EDIT_POSITION) {
    setXyzStatus("TCP 平移：拖动三个正向箭头，半透明机械臂实时 IK；松开后规划轨迹。", "ready");
  } else if (targetPoseEditor?.state === TARGET_EDIT_STATE.EDIT_ORIENTATION) {
    setXyzStatus("TCP 旋转：拖动局部 XYZ 圆环，半透明机械臂实时 IK；松开后规划轨迹。", "ready");
  } else if (cartesianPlaneKey) {
    const plane = currentCartesianPlane();
    setXyzStatus(
      `已选择基座 ${plane?.name ?? "—"} 平面：保持按键并拖动白色末端。`,
      "ready",
    );
  } else if (previewKind === "park") {
    setXyzStatus("回参考 Ghost 与末端轨迹已显示；再次点击顶部按钮确认执行。", "ready");
  } else if (!deltaValid) {
    setXyzStatus("目标与当前 TCP 基本一致；可直接拖动平移箭头或旋转环。", "");
  } else if (previewKind !== "cartesian") {
    setXyzStatus("目标尚未验证；点击“预览 XYZ”不会移动真机。", "");
  }
}

function setControlMode(mode) {
  if (!CONTROL_MODES.includes(mode)) return;
  if ((armState.busy || commandPending || parkControlFollowing)
      && mode !== activeControlMode) return;
  if (mode !== activeControlMode && previewDirty) resetPreview({ quiet: true });
  if (mode !== "xyz") {
    if (cartesianDrag) cancelCartesianDrag("已取消末端拖动。");
    stopServoHeartbeat();
    elements.servoEnabled.checked = false;
    cartesianPlaneKey = null;
    setEefHandlePlane();
    elements.canvas.classList.remove("xyz-drag-ready", "xyz-drag-active");
  }
  if (mode === "hand") stopGripperKeyControl();
  activeControlMode = mode;
  teleoperationWorkspaceBox.visible = mode === "hand"
    && teleoperationWorkspaceBox.userData.configured;
  updateModelHighlight();
  elements.controlModes.dataset.selected = mode;
  if (mode !== "joint" && elements.advancedJoints) elements.advancedJoints.open = false;
  elements.xyzPane.hidden = mode !== "xyz";
  elements.jointPane.hidden = mode !== "joint";
  elements.handPane.hidden = mode !== "hand";
  elements.speedControl.hidden = mode === "hand";
  elements.controlModeButtons.forEach((button) => {
    const selected = button.dataset.mode === mode;
    button.classList.toggle("selected", selected);
    button.setAttribute("aria-pressed", selected ? "true" : "false");
  });
  if (selectionRing) selectionRing.visible = modelsReady && mode === "joint";
  if (eefDragHandle) {
    eefDragHandle.visible = modelsReady && mode === "xyz"
      && tcpControlsVisible
      && elements.servoEnabled.checked;
  }
  if (targetPoseEditor) {
    targetPoseEditor.setModeActive(mode === "xyz");
    targetPoseEditor.setControlsVisible(tcpControlsVisible);
  }
  if (targetGripperGhost) targetGripperGhost.root.visible = mode === "xyz" && Boolean(targetPose);
  updateCartesianInspector();
  updateInspector();
  window.dispatchEvent(new CustomEvent("arx5-teleoperation-mode", {
    detail: { active: mode === "hand" },
  }));
}

function setPreviewTarget(requestedTarget) {
  if (!motionAvailable()) return;
  const window = safeTargetWindow();
  if (window === null || !Number.isFinite(requestedTarget)) return;
  const target = Math.max(window.minimum, Math.min(window.maximum, requestedTarget));
  previewAngles = actualAngles.slice();
  previewAngles[selectedJoint] = target;
  previewDirty = Math.abs(target - actualAngles[selectedJoint]) >= 0.1;
  previewKind = previewDirty ? "joint" : null;
  cartesianPreview = null;
  displayedPreviewAngles = previewAngles.slice();
  if (previewModel) setModelAngles(previewModel, previewAngles);
  setPlannedPath(
    previewDirty ? jointEndEffectorPathMm(actualAngles, previewAngles) : null,
    "joint",
  );
  updatePreviewVisibility();
  updateInspector();
  if (previewDirty) {
    const clamped = Math.abs(target - requestedTarget) > 1e-6;
    setPreviewStatus(
      clamped
        ? `已限制到软件关节范围：${target.toFixed(1)}°。`
        : `J${selectedJoint + 1} 目标已预览；确认模型姿态后再执行。`,
      "ready",
    );
  } else {
    setPreviewStatus("预览与当前反馈姿态一致。", "");
  }
}

function updateInspector() {
  const hasFeedback = Boolean(armState.connected)
    && finiteJointVector(armState.joint_position_deg);
  const actual = hasFeedback ? actualAngles[selectedJoint] : Number.NaN;
  const target = hasFeedback ? previewAngles[selectedJoint] : Number.NaN;
  const delta = target - actual;
  const window = safeTargetWindow();
  const available = modelsReady && motionAvailable() && window !== null;

  elements.actual.textContent = Number.isFinite(actual) ? `${actual.toFixed(1)}°` : "—°";
  elements.target.textContent = Number.isFinite(target) ? `${target.toFixed(1)}°` : "—°";
  elements.delta.textContent = Number.isFinite(delta)
    ? `${delta >= 0 ? "+" : ""}${delta.toFixed(1)}°`
    : "—°";
  elements.dialNeedle.style.transform = `rotate(${Number.isFinite(target) ? target : 0}deg)`;
  elements.dial.classList.toggle("disabled", !available);
  elements.dial.tabIndex = available ? 0 : -1;

  if (window) {
    elements.limit.textContent = `J${selectedJoint + 1} ${JOINT_NAMES[selectedJoint]} · 本次 ${window.minimum.toFixed(1)}° … ${window.maximum.toFixed(1)}°`;
    elements.targetInput.min = window.minimum.toFixed(1);
    elements.targetInput.max = window.maximum.toFixed(1);
    elements.targetInput.value = target.toFixed(1);
    elements.dial.setAttribute("aria-valuemin", window.minimum.toFixed(1));
    elements.dial.setAttribute("aria-valuemax", window.maximum.toFixed(1));
    elements.dial.setAttribute("aria-valuenow", target.toFixed(1));
    elements.dial.setAttribute("aria-valuetext", `J${selectedJoint + 1} ${target.toFixed(1)} 度`);
  } else {
    elements.limit.textContent = "连接机械臂后显示软件关节范围。";
    elements.targetInput.value = "";
  }

  elements.targetInput.disabled = !available;
  elements.nudgeButtons.forEach((button) => { button.disabled = !available; });
  elements.previewResetButton.disabled = !previewDirty || armState.busy || commandPending;
  elements.syncPreviewButton.disabled = !previewDirty || armState.busy || commandPending;
  elements.executeButton.disabled = !available || previewKind !== "joint" || !previewDirty;
  if (selectionRing) {
    selectionRing.visible = modelsReady && activeControlMode === "joint";
    selectionRing.traverse((object) => {
      if (object.material && object.material.opacity > 0.01) {
        object.material.opacity = available ? 0.9 : 0.35;
      }
    });
  }

  if (!armState.connected) {
    setPreviewStatus("3D 模型仅显示，尚未连接机械臂。", "");
  } else if (armState.mode === "fault") {
    setPreviewStatus("当前为故障阻尼；请先检查故障原因。", "error");
  } else if (armState.busy || commandPending) {
    setPreviewStatus("机械臂正在执行命令，请等待反馈。", "");
  } else if (directKeyHeld && !ringDrag) {
    setPreviewStatus("Space 快捷拖动已启用：按住空格，从任意连杆开始拖动。", "ready");
  } else if (previewKind === "park") {
    setPreviewStatus("回参考 Ghost 与末端轨迹已显示；再次点击顶部按钮确认执行。", "ready");
  } else if (!previewDirty) {
    setPreviewStatus("点击连杆选择关节，再拖动绿色圆环或右侧旋钮。", "");
  }
}

function handleArmState(nextState) {
  const hadLiveFeedback = Boolean(armState.connected)
    && finiteJointVector(armState.joint_position_deg);
  const previousConnected = Boolean(armState.connected);
  const previousBusy = Boolean(armState.busy);
  const previousServoActive = Boolean(armState.servo_active);
  const previousMode = armState.mode;
  armState = nextState;
  refreshHandeyeBoardPreview();
  if (plannedPathExecutionActive) plannedPathFeedbackDirty = true;
  updateTeleoperationWorkspaceBox(armState.teleoperation_config);
  const cubeStage = armState.cube_manipulation?.stage ?? "idle";
  if (["held", "over_target"].includes(cubeStage)
      && !carriedCubeAttached
      && !carriedCubeReleasedFromGripper) {
    attachCarriedCube({ force: true });
  } else if (cubeStage === "released" && carriedCubeAttached) {
    detachCarriedCube();
  }
  const enabledCamera = nextState.handeye_validation?.enabled
    ? nextState.handeye_validation.T_base_camera : null;
  const enabledD405 = nextState.handeye_validation?.enabled
    ? nextState.handeye_validation.T_eef_camera : null;
  if (enabledCamera) applyHandeyeCameraTransform(enabledCamera);
  else if (handeyeCameraTransform) {
    handeyeCameraTransform = null;
    applyCameraCalibration(cameraCalibrationState, { persist: false, syncInputs: true });
  }
  applyHandeyeD405Transform(enabledD405);
  const actualTcpPose = actualTargetPoseFromState(armState);
  if (targetPoseEditor) {
    targetPoseEditor.setActualPose(actualTcpPose);
    if (parkControlFollowing && actualTcpPose) {
      targetPoseEditor.updateFeedbackFollow(actualTcpPose);
    }
    const enteredDamping = previousMode !== "damping" && armState.mode === "damping";
    const followDampingTcp = armState.mode === "damping"
      && !previewDirty
      && !commandPending
      && !targetPoseEditor.drag
      && (enteredDamping || targetPoseEditor.state === TARGET_EDIT_STATE.IDLE);
    if (actualTcpPose && (followDampingTcp || !hadLiveFeedback || !targetPose)) {
      targetPose = cloneTargetPose(actualTcpPose);
      if (followDampingTcp) targetPoseEditor.syncToActualPose(targetPose);
      else {
        targetPoseEditor.setTargetPose(targetPose, {
          resetRoll: true,
          markReady: false,
        });
      }
      updateTargetPoseReadout();
    }
  }
  const measuredGripperWidth = Number(armState.gripper_position_mm);
  if (Number.isFinite(measuredGripperWidth)) {
    actualGripperWidthMm = measuredGripperWidth;
    if (elements.gripperWidth.dataset.previewDirty !== "true"
        && !hasTeleoperationPreview()) {
      targetGripperWidthMm = measuredGripperWidth;
      if (targetPose) targetPose.gripperWidth = measuredGripperWidth / 1000.0;
      targetPoseEditor?.setGripperWidth(measuredGripperWidth / 1000.0);
    }
    if (!hadLiveFeedback) displayedGripperWidthMm = measuredGripperWidth;
  }
  if (armState.connected && finiteJointVector(armState.joint_position_deg)) {
    actualAngles = armState.joint_position_deg.map(Number);
    if (!hadLiveFeedback) displayedAngles = actualAngles.slice();
    if (!previewDirty) {
      previewAngles = actualAngles.slice();
      displayedPreviewAngles = previewAngles.slice();
    } else if (hasFullRobotPreview()) {
      previewAngles = previewAngles.slice();
    } else {
      previewAngles = actualAngles.map((value, index) => (
        index === selectedJoint ? previewAngles[index] : value
      ));
      displayedPreviewAngles = previewAngles.slice();
    }
    if (previewModel?.root.visible && !hasFullRobotPreview()) {
      setModelAngles(previewModel, displayedPreviewAngles);
    }
  } else if (!armState.connected) {
    parkControlFollowing = false;
    elements.servoEnabled.checked = false;
    targetPoseEditor?.endFeedbackFollow();
    targetPose = null;
    actualAngles = DISCONNECTED_DISPLAY_ANGLES.slice();
    displayedAngles = actualAngles.slice();
    actualGripperWidthMm = DISCONNECTED_GRIPPER_WIDTH_MM;
    displayedGripperWidthMm = DISCONNECTED_GRIPPER_WIDTH_MM;
    targetGripperWidthMm = DISCONNECTED_GRIPPER_WIDTH_MM;
    displayedTargetGripperWidthMm = DISCONNECTED_GRIPPER_WIDTH_MM;
    delete elements.gripperWidth.dataset.previewDirty;
    resetPreview({ quiet: true });
    setCartesianGuide(null, null);
    targetPoseEditor?.clearPose();
    if (actualModel) {
      setModelAngles(actualModel, displayedAngles);
      setModelGripperWidth(actualModel, displayedGripperWidthMm);
    }
    if (previewModel) {
      setModelAngles(previewModel, displayedPreviewAngles);
      setModelGripperWidth(previewModel, displayedGripperWidthMm);
    }
  }
  if (armState.connected && parkControlFollowing
      && !armState.busy && !commandPending) {
    endMotionFeedbackFollow();
  }
  const now = performance.now();
  const controlStateChanged = previousConnected !== Boolean(armState.connected)
    || previousBusy !== Boolean(armState.busy)
    || previousServoActive !== Boolean(armState.servo_active)
    || previousMode !== armState.mode;
  if (controlStateChanged || now - lastArmStateUiUpdateAt >= ARM_STATE_UI_INTERVAL_MS) {
    lastArmStateUiUpdateAt = now;
    updateTargetGhost(targetPose);
    updatePreviewVisibility();
    updateInspector();
    updateCartesianInspector();
  }
}

function pointerToNdc(event) {
  const rect = elements.canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
}

function currentEefWorldPosition() {
  if (!eefDragHandle) return null;
  return eefDragHandle.getWorldPosition(new THREE.Vector3());
}

function modelEefWorldPosition(model) {
  const link6 = model?.motions?.[5];
  if (!link6) return null;
  model.root.updateMatrixWorld(true);
  return link6.localToWorld(new THREE.Vector3(EEF_OFFSET_LINK6_M, 0, 0));
}

function jointEndEffectorPathMm(startAngles, targetAngles, sampleCount = 48) {
  if (!previewModel || !finiteJointVector(startAngles) || !finiteJointVector(targetAngles)) {
    return null;
  }
  const restoreAngles = finiteJointVector(displayedPreviewAngles)
    ? displayedPreviewAngles.slice()
    : targetAngles.slice();
  const path = [];
  for (let sample = 0; sample < sampleCount; sample += 1) {
    const amount = sample / Math.max(1, sampleCount - 1);
    const angles = startAngles.map((start, index) => (
      start + (targetAngles[index] - start) * amount
    ));
    setModelAngles(previewModel, angles);
    const point = modelEefWorldPosition(previewModel);
    if (point) path.push(point.toArray().map((value) => value * 1000));
  }
  setModelAngles(previewModel, restoreAngles);
  return path.length >= 2 ? path : null;
}

function clampCartesianPlaneMovement(startValues, movementMm) {
  const axes = ["x", "y", "z"];
  return Object.fromEntries(axes.map((axis) => [
    `${axis}_mm`,
    startValues[`${axis}_mm`] + (movementMm[axis] ?? 0),
  ]));
}

function beginCartesianDrag(event) {
  const plane = currentCartesianPlane();
  if (activeControlMode !== "xyz" || !plane || !motionAvailable()
      || !actualModel || !eefDragHandle) return false;
  pointerToNdc(event);
  const terminalLink = actualModel.meshes[actualModel.meshes.length - 1];
  const hit = raycaster.intersectObjects([eefDragHandle, terminalLink], false)[0];
  if (!hit) return false;

  const startWorld = currentEefWorldPosition();
  const viewAlignment = Math.abs(raycaster.ray.direction.dot(plane.normal));
  if (viewAlignment < 0.08) {
    setXyzStatus(
      `${plane.name} 平面当前几乎侧对相机，请先稍微旋转 3D 视角。`,
      "error",
    );
    event.preventDefault();
    event.stopImmediatePropagation();
    return true;
  }
  const movementPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(
    plane.normal,
    startWorld,
  );
  const pointerStartWorld = raycaster.ray.intersectPlane(
    movementPlane,
    new THREE.Vector3(),
  );
  if (!pointerStartWorld) return false;

  const liveServo = liveServoEnabled();
  if (previewDirty) resetPreview({ quiet: true });
  const startInputValues = liveServo
    ? { x_mm: 0, y_mm: 0, z_mm: 0 }
    : requestedCartesianDelta();
  if (liveServo) {
    Object.values(elements.xyzInputs).forEach((input) => { input.value = "0"; });
    if (sendServoMessage("start") === null) {
      setXyzStatus("无法发送实时伺服启动消息。", "error");
      return true;
    }
  }
  cartesianDrag = {
    generation: ++cartesianDragSequence,
    pointerId: event.pointerId,
    plane,
    movementPlane,
    pointerStartWorld,
    startWorld,
    startInputValues,
    startAngles: actualAngles.slice(),
    previewInitialized: false,
    lastValidDelta: null,
    liveServo,
  };
  previewKind = liveServo ? "cartesian-servo" : "cartesian-live";
  previewDirty = true;
  previewAngles = actualAngles.slice();
  displayedPreviewAngles = previewAngles.slice();
  if (previewModel) setModelAngles(previewModel, previewAngles);
  updatePreviewVisibility();
  controls.enabled = false;
  elements.canvas.classList.add("xyz-drag-active");
  elements.canvas.setPointerCapture(event.pointerId);
  setCartesianGuide(
    startWorld,
    startWorld.clone().add(new THREE.Vector3(
      startInputValues.x_mm / 1000.0,
      startInputValues.y_mm / 1000.0,
      startInputValues.z_mm / 1000.0,
    )),
    plane.color,
  );
  if (liveServo) startServoHeartbeat();
  else scheduleLiveCartesianPreview();
  updateCartesianInspector();
  setXyzStatus(
    liveServo
      ? `实时伺服：正在基座 ${plane.name} 平面控制真机；松开即平滑停车。`
      : `正在基座 ${plane.name} 平面拖动；半透明模型实时显示 IK。`,
    "ready",
  );
  event.preventDefault();
  event.stopImmediatePropagation();
  return true;
}

function moveCartesianDrag(event) {
  if (!cartesianDrag || cartesianDrag.pointerId !== event.pointerId) return false;
  pointerToNdc(event);
  const pointerWorld = raycaster.ray.intersectPlane(
    cartesianDrag.movementPlane,
    dragPoint,
  );
  if (!pointerWorld) return false;
  const worldMovement = pointerWorld.clone().sub(cartesianDrag.pointerStartWorld);
  const movementMm = { x: 0, y: 0, z: 0 };
  cartesianDrag.plane.axes.forEach((axis) => {
    movementMm[axis] = Math.round(
      (1000 * worldMovement[axis]) / CARTESIAN_DRAG_SNAP_MM,
    ) * CARTESIAN_DRAG_SNAP_MM;
  });
  const startValues = cartesianDrag.startInputValues;
  const clamped = clampCartesianPlaneMovement(startValues, movementMm);
  elements.xyzInputs.x.value = clamped.x_mm.toFixed(1);
  elements.xyzInputs.y.value = clamped.y_mm.toFixed(1);
  elements.xyzInputs.z.value = clamped.z_mm.toFixed(1);
  const combinedDelta = requestedCartesianDelta();
  const target = cartesianDrag.startWorld.clone().add(
    new THREE.Vector3(
      combinedDelta.x_mm / 1000.0,
      combinedDelta.y_mm / 1000.0,
      combinedDelta.z_mm / 1000.0,
    ),
  );
  setCartesianGuide(
    cartesianDrag.startWorld,
    target,
    cartesianDrag.plane.color,
  );
  updateCartesianInspector();
  const [axisA, axisB] = cartesianDrag.plane.axes;
  const valueA = combinedDelta[`${axisA}_mm`];
  const valueB = combinedDelta[`${axisB}_mm`];
  setXyzStatus(
    `${cartesianDrag.plane.name} · ${axisA.toUpperCase()} ${valueA >= 0 ? "+" : ""}${valueA.toFixed(1)} · ${axisB.toUpperCase()} ${valueB >= 0 ? "+" : ""}${valueB.toFixed(1)} mm · 合成 ${Math.hypot(combinedDelta.x_mm, combinedDelta.y_mm, combinedDelta.z_mm).toFixed(1)} mm`,
    "ready",
  );
  if (cartesianDrag.liveServo) sendLiveServoTarget();
  else scheduleLiveCartesianPreview();
  event.preventDefault();
  event.stopImmediatePropagation();
  return true;
}

function releaseCartesianPointer() {
  if (!cartesianDrag) return;
  const pointerId = cartesianDrag.pointerId;
  controls.enabled = true;
  elements.canvas.classList.remove("xyz-drag-active");
  if (elements.canvas.hasPointerCapture(pointerId)) {
    elements.canvas.releasePointerCapture(pointerId);
  }
}

function cancelCartesianDrag(message = "已取消末端拖动。", restoreInputs = true) {
  if (!cartesianDrag) return false;
  const previous = cartesianDrag.startInputValues;
  const liveServo = cartesianDrag.liveServo;
  if (liveServo) {
    stopServoHeartbeat();
    sendServoMessage("stop");
  }
  releaseCartesianPointer();
  cartesianDrag = undefined;
  clearQueuedLiveCartesianPreview();
  if (liveServo) {
    Object.values(elements.xyzInputs).forEach((input) => { input.value = "0"; });
  } else if (restoreInputs && previous) {
    elements.xyzInputs.x.value = String(previous.x_mm);
    elements.xyzInputs.y.value = String(previous.y_mm);
    elements.xyzInputs.z.value = String(previous.z_mm);
  }
  if (hasCartesianVisualPreview()) resetPreview({ quiet: true });
  else {
    setCartesianGuide(null, null);
    updateCartesianInspector();
  }
  setXyzStatus(message, "");
  return true;
}

function endCartesianDrag(event) {
  if (!cartesianDrag || cartesianDrag.pointerId !== event.pointerId) return false;
  const finishedDrag = cartesianDrag;
  const delta = requestedCartesianDelta();
  const previous = finishedDrag.startInputValues;
  releaseCartesianPointer();
  cartesianDrag = undefined;
  clearQueuedLiveCartesianPreview();
  event.preventDefault();
  event.stopImmediatePropagation();
  if (finishedDrag.liveServo) {
    stopServoHeartbeat();
    sendServoMessage("stop");
    Object.values(elements.xyzInputs).forEach((input) => { input.value = "0"; });
    if (previewKind === "cartesian-servo"
        || previewKind === "cartesian-servo-limit") {
      resetPreview({ quiet: true });
    }
    else {
      setCartesianGuide(null, null);
      updateCartesianInspector();
    }
    setXyzStatus("已释放 deadman，正在平滑停车。", "ready");
    return true;
  }
  if (!validCartesianDelta(delta)) {
    if (previous) {
      elements.xyzInputs.x.value = String(previous.x_mm);
      elements.xyzInputs.y.value = String(previous.y_mm);
      elements.xyzInputs.z.value = String(previous.z_mm);
    }
    setCartesianGuide(null, null);
    updateCartesianInspector();
    setXyzStatus("拖动距离太小，没有生成预览。", "");
    return true;
  }
  requestCartesianPreview({
    fallbackDelta: finishedDrag.lastValidDelta,
    dragGeneration: finishedDrag.generation,
    dragSource: "cartesian-plane",
  });
  return true;
}

function ringPointerAngle(event) {
  pointerToNdc(event);
  if (!raycaster.ray.intersectPlane(dragPlane, dragPoint)) return null;
  dragPoint.sub(dragOrigin);
  if (dragPoint.lengthSq() < 1e-7) return null;
  return Math.atan2(dragPoint.dot(dragBasisY), dragPoint.dot(dragBasisX));
}

function beginJointRotationDrag(event, direct) {
  const worldQuaternion = selectionRing.getWorldQuaternion(new THREE.Quaternion());
  selectionRing.getWorldPosition(dragOrigin);
  dragAxis.set(0, 0, 1).applyQuaternion(worldQuaternion).normalize();
  dragBasisX.set(1, 0, 0).applyQuaternion(worldQuaternion).normalize();
  dragBasisY.set(0, 1, 0).applyQuaternion(worldQuaternion).normalize();
  dragPlane.setFromNormalAndCoplanarPoint(dragAxis, dragOrigin);
  const angle = ringPointerAngle(event);
  if (angle === null) return false;

  ringDrag = {
    pointerId: event.pointerId,
    startAngle: angle,
    startTarget: previewAngles[selectedJoint],
    direct,
  };
  controls.enabled = false;
  elements.canvas.classList.toggle("direct-drag-active", direct);
  elements.canvas.setPointerCapture(event.pointerId);
  event.preventDefault();
  event.stopImmediatePropagation();
  return true;
}

function beginRingDrag(event, direct = false) {
  if (activeControlMode !== "joint" || !motionAvailable() || !selectionRing) return false;
  pointerToNdc(event);
  const hits = raycaster.intersectObjects(ringHitTargets, false);
  if (hits.length === 0) return false;
  return beginJointRotationDrag(event, direct);
}

function beginDirectLinkDrag(event) {
  if (activeControlMode !== "joint" || !directKeyHeld || !motionAvailable() || !actualModel) return false;
  pointerToNdc(event);
  const hit = raycaster.intersectObjects(actualModel.meshes.slice(1), false)[0];
  const jointIndex = hit?.object.userData.jointIndex;
  if (!Number.isInteger(jointIndex)) return false;
  selectJoint(jointIndex);
  return beginJointRotationDrag(event, true);
}

function moveRingDrag(event) {
  if (!ringDrag || ringDrag.pointerId !== event.pointerId) return;
  const angle = ringPointerAngle(event);
  if (angle === null) return;
  let delta = angle - ringDrag.startAngle;
  if (delta > Math.PI) delta -= 2 * Math.PI;
  if (delta < -Math.PI) delta += 2 * Math.PI;
  setPreviewTarget(ringDrag.startTarget + THREE.MathUtils.radToDeg(delta));
  if (ringDrag.direct && previewDirty) {
    setPreviewStatus(
      `Space 拖动 J${selectedJoint + 1}：松开鼠标自动执行；先松空格取消。`,
      "ready",
    );
  }
  event.preventDefault();
  event.stopImmediatePropagation();
}

function endRingDrag(event) {
  if (!ringDrag || ringDrag.pointerId !== event.pointerId) return false;
  const direct = ringDrag.direct;
  ringDrag = undefined;
  controls.enabled = true;
  elements.canvas.classList.remove("direct-drag-active");
  if (elements.canvas.hasPointerCapture(event.pointerId)) {
    elements.canvas.releasePointerCapture(event.pointerId);
  }
  event.preventDefault();
  event.stopImmediatePropagation();
  if (direct && directKeyHeld && previewDirty) executePreview();
  return true;
}

function cancelDirectDrag() {
  if (!ringDrag?.direct) return;
  const pointerId = ringDrag.pointerId;
  ringDrag = undefined;
  controls.enabled = true;
  elements.canvas.classList.remove("direct-drag-active");
  if (elements.canvas.hasPointerCapture(pointerId)) {
    elements.canvas.releasePointerCapture(pointerId);
  }
  resetPreview({ quiet: true });
  setPreviewStatus("已取消 Space 拖动，没有发送运动命令。", "");
}

function dialPointerAngle(event) {
  const rect = elements.dial.getBoundingClientRect();
  return Math.atan2(
    event.clientX - (rect.left + rect.width / 2),
    -(event.clientY - (rect.top + rect.height / 2)),
  );
}

function normalizedAngleDelta(value) {
  if (value > Math.PI) return value - 2 * Math.PI;
  if (value < -Math.PI) return value + 2 * Math.PI;
  return value;
}

elements.dial.addEventListener("pointerdown", (event) => {
  if (!motionAvailable() || safeTargetWindow() === null) return;
  dialDrag = {
    pointerId: event.pointerId,
    startAngle: dialPointerAngle(event),
    startTarget: previewAngles[selectedJoint],
  };
  elements.dial.setPointerCapture(event.pointerId);
  event.preventDefault();
});

elements.dial.addEventListener("pointermove", (event) => {
  if (!dialDrag || dialDrag.pointerId !== event.pointerId) return;
  const delta = normalizedAngleDelta(dialPointerAngle(event) - dialDrag.startAngle);
  setPreviewTarget(dialDrag.startTarget + THREE.MathUtils.radToDeg(delta));
});

function finishDialDrag(event) {
  if (!dialDrag || dialDrag.pointerId !== event.pointerId) return;
  dialDrag = undefined;
  if (elements.dial.hasPointerCapture(event.pointerId)) {
    elements.dial.releasePointerCapture(event.pointerId);
  }
}

elements.dial.addEventListener("pointerup", finishDialDrag);
elements.dial.addEventListener("pointercancel", finishDialDrag);
elements.dial.addEventListener("keydown", (event) => {
  const step = event.shiftKey ? 5 : 1;
  if (event.key === "ArrowLeft" || event.key === "ArrowDown") {
    setPreviewTarget(previewAngles[selectedJoint] - step);
    event.preventDefault();
  } else if (event.key === "ArrowRight" || event.key === "ArrowUp") {
    setPreviewTarget(previewAngles[selectedJoint] + step);
    event.preventDefault();
  } else if (event.key === "Escape") {
    resetPreview();
  }
});

elements.canvas.addEventListener("pointerdown", (event) => {
  elements.canvas.focus({ preventScroll: true });
  if (activeControlMode === "xyz" && targetPoseEditor?.pointerDown(event)) {
    event.preventDefault();
    event.stopImmediatePropagation();
    return;
  }
  if (activeControlMode === "xyz" && cartesianPlaneKey) {
    if (beginCartesianDrag(event)) return;
    const plane = currentCartesianPlane();
    setXyzStatus(
      `按住 ${cartesianPlaneKey.toUpperCase()}（${plane?.name ?? "—"} 平面）后，请从白色末端开始拖动。`,
      "error",
    );
    event.preventDefault();
    event.stopImmediatePropagation();
    return;
  }
  if (directKeyHeld) {
    if (beginDirectLinkDrag(event) || beginRingDrag(event, true)) return;
    setPreviewStatus("Space 拖动需要从机械臂连杆或绿色关节圆环开始。", "error");
    event.preventDefault();
    event.stopImmediatePropagation();
    return;
  }
  if (beginRingDrag(event, false)) return;
  canvasPointerDown = { x: event.clientX, y: event.clientY, pointerId: event.pointerId };
}, true);

elements.canvas.addEventListener("pointermove", (event) => {
  if (targetPoseEditor?.pointerMove(event)) {
    event.preventDefault();
    event.stopImmediatePropagation();
    return;
  }
  if (moveCartesianDrag(event)) return;
  moveRingDrag(event);
  updateCubeSelectionCursor(event);
}, true);
elements.canvas.addEventListener("pointerup", (event) => {
  if (targetPoseEditor?.pointerUp(event)) {
    event.preventDefault();
    event.stopImmediatePropagation();
    return;
  }
  if (endCartesianDrag(event)) return;
  if (endRingDrag(event)) return;
  if (!canvasPointerDown || canvasPointerDown.pointerId !== event.pointerId || !actualModel) return;
  const travel = Math.hypot(
    event.clientX - canvasPointerDown.x,
    event.clientY - canvasPointerDown.y,
  );
  canvasPointerDown = undefined;
  if (travel > 6) return;
  pointerToNdc(event);
  const cubeCandidateId = detectedCubeCandidateAtPointer();
  if (cubeCandidateId !== null) {
    window.dispatchEvent(new CustomEvent("arx5-cube-select", {
      detail: { candidate_id: cubeCandidateId, source: "simulator" },
    }));
    event.preventDefault();
    return;
  }
  const hit = raycaster.intersectObjects(actualModel.meshes.slice(1), false)[0];
  if (activeControlMode === "joint" && Number.isInteger(hit?.object.userData.jointIndex)) {
    selectJoint(hit.object.userData.jointIndex);
  }
}, true);
elements.canvas.addEventListener("pointercancel", (event) => {
  if (targetPoseEditor?.drag) {
    targetPoseEditor.cancelPointer();
    event.preventDefault();
    return;
  }
  if (cartesianDrag) cancelCartesianDrag("指针操作中断，已取消末端拖动。");
  else if (ringDrag?.direct) cancelDirectDrag();
  else endRingDrag(event);
  canvasPointerDown = undefined;
}, true);

elements.canvas.addEventListener("mouseenter", () => {
  canvasHovered = true;
  elements.canvas.classList.toggle("direct-drag-ready", directKeyHeld);
  elements.canvas.classList.toggle("xyz-drag-ready", Boolean(cartesianPlaneKey));
});
elements.canvas.addEventListener("mouseleave", () => {
  canvasHovered = false;
  elements.canvas.classList.remove("cube-selectable");
  if (!ringDrag) elements.canvas.classList.remove("direct-drag-ready");
  if (!cartesianDrag) elements.canvas.classList.remove("xyz-drag-ready");
});

function shortcutMayCaptureCanvas() {
  const active = document.activeElement;
  const editable = active instanceof HTMLElement
    && (active.isContentEditable
      || ["INPUT", "SELECT", "TEXTAREA", "BUTTON", "A"].includes(active.tagName));
  const shortcutDockFocused = active instanceof HTMLElement
    && Boolean(active.closest(".stage-shortcuts"));
  return !editable && (canvasHovered || active === elements.canvas || shortcutDockFocused);
}

function shortcutMayCaptureEmergencyStop() {
  const active = document.activeElement;
  return !(active instanceof HTMLElement
    && (active.isContentEditable || ["INPUT", "SELECT", "TEXTAREA"].includes(active.tagName)));
}

function setTcpControlsVisible(visible, notify = true) {
  tcpControlsVisible = Boolean(visible);
  targetPoseEditor?.setControlsVisible(tcpControlsVisible);
  if (eefDragHandle) {
    eefDragHandle.visible = modelsReady
      && activeControlMode === "xyz"
      && tcpControlsVisible
      && elements.servoEnabled.checked;
  }
  updateCartesianInspector();
  if (notify) {
    setShortcutStatus(
      tcpControlsVisible ? "TCP 控件已显示。" : "TCP 控件已隐藏；按 V 恢复。",
      "ready",
    );
  }
}

window.addEventListener("keydown", (event) => {
  if (event.code === "Escape"
      && (tableCalibrationEditing || cameraCalibrationEditing || lightCalibrationEditing)) {
    setTableCalibrationEditing(false);
    setCameraCalibrationEditing(false);
    setLightCalibrationEditing(false);
    elements.canvas.focus({ preventScroll: true });
    event.preventDefault();
    return;
  }
  if (activeControlMode === "xyz" && shortcutMayCaptureCanvas()
      && targetPoseEditor?.handleShortcut(event)) {
    event.preventDefault();
    event.stopImmediatePropagation();
    return;
  }
  if (event.code === "Escape" && (cartesianDrag || ringDrag?.direct || previewDirty)) {
    const wasCartesian = activeControlMode === "xyz";
    if (cartesianDrag) cancelCartesianDrag("已取消末端拖动。");
    cancelDirectDrag();
    if (previewDirty) resetPreview({ quiet: true });
    cartesianPlaneKey = null;
    setEefHandlePlane();
    elements.canvas.classList.remove("xyz-drag-ready", "xyz-drag-active");
    if (wasCartesian) setXyzStatus("已取消 XYZ 预览，没有发送运动命令。", "");
    else setPreviewStatus("已取消关节预览，没有发送运动命令。", "");
    event.preventDefault();
    return;
  }
  if (event.code === "KeyM" && !event.metaKey && !event.ctrlKey && !event.altKey
      && shortcutMayCaptureCanvas()) {
    event.preventDefault();
    if (event.repeat || cartesianDrag || ringDrag) return;
    const nextMode = CONTROL_MODES[
      (CONTROL_MODES.indexOf(activeControlMode) + 1) % CONTROL_MODES.length
    ];
    setControlMode(nextMode);
    return;
  }

  if (activeControlMode !== "hand"
      && event.code === "KeyV" && !event.metaKey && !event.ctrlKey && !event.altKey
      && shortcutMayCaptureEmergencyStop()) {
    event.preventDefault();
    if (!event.repeat) setTcpControlsVisible(!tcpControlsVisible);
    return;
  }

  if (activeControlMode !== "hand"
      && event.code === "KeyN" && !event.metaKey && !event.ctrlKey && !event.altKey
      && shortcutMayCaptureCanvas()) {
    event.preventDefault();
    if (event.repeat) return;
    if (!motionAvailable() || cartesianDrag || ringDrag) {
      setShortcutStatus("当前状态不能回参考。", "error");
      return;
    }
    if (!Array.isArray(armState.reference_joint_position_deg)) {
      setShortcutStatus("尚未记录参考姿态。", "error");
      return;
    }
    if (previewDirty && previewKind !== "park") resetPreview({ quiet: true });
    cartesianPlaneKey = null;
    setEefHandlePlane();
    elements.canvas.classList.remove("xyz-drag-ready");
    const stage = requestParkAction();
    if (stage === "preview") {
      setShortcutStatus("N：正在生成回参考 Ghost 与末端轨迹…", "ready");
    } else if (stage === "execute") {
      setShortcutStatus("N：正在执行已确认的回参考轨迹，完成后主动保持。", "ready");
    }
    return;
  }

  if (activeControlMode !== "hand" && (event.code === "KeyD" || event.code === "KeyF")
      && !event.metaKey && !event.ctrlKey && !event.altKey
      && shortcutMayCaptureCanvas()) {
    event.preventDefault();
    if (event.repeat) return;
    gripperKeysHeld.add(event.code);
    updateGripperKeyDirection(event.code === "KeyD" ? 1 : -1);
    return;
  }

  if (event.code !== "Space" || !shortcutMayCaptureEmergencyStop()) return;
  event.preventDefault();
  if (event.repeat) return;
  stopGripperKeyControl();
  targetPoseEditor?.cancelPointer();
  if (cartesianDrag) cancelCartesianDrag("已发送急停，末端拖动已停止。", false);
  if (ringDrag) {
    const pointerId = ringDrag.pointerId;
    ringDrag = undefined;
    controls.enabled = true;
    elements.canvas.classList.remove("direct-drag-active");
    if (elements.canvas.hasPointerCapture(pointerId)) {
      elements.canvas.releasePointerCapture(pointerId);
    }
    resetPreview({ quiet: true });
  }
  directKeyHeld = false;
  elements.canvas.classList.remove("direct-drag-ready");
  cartesianPlaneKey = null;
  setEefHandlePlane();
  elements.canvas.classList.remove("xyz-drag-ready", "xyz-drag-active");
  requestEmergencyStop();
  setShortcutStatus("Space 急停已发送，机械臂正在进入阻尼。", "error");
});

window.addEventListener("keyup", (event) => {
  if ((event.code === "KeyD" || event.code === "KeyF")
      && (gripperKeysHeld.has(event.code) || gripperServoDirection !== 0)) {
    event.preventDefault();
    gripperKeysHeld.delete(event.code);
    if (gripperKeysHeld.size === 0) {
      stopGripperKeyControl();
    } else {
      updateGripperKeyDirection(gripperKeysHeld.has("KeyF") ? -1 : 1);
    }
    return;
  }
  if (event.code !== "Space") return;
  event.preventDefault();
});

window.addEventListener("blur", () => {
  stopGripperKeyControl();
  targetPoseEditor?.cancelPointer();
  directKeyHeld = false;
  cartesianPlaneKey = null;
  setEefHandlePlane();
  elements.canvas.classList.remove("direct-drag-ready");
  elements.canvas.classList.remove("xyz-drag-ready", "xyz-drag-active");
  cancelCartesianDrag("浏览器失去焦点，已取消末端拖动。");
  cancelDirectDrag();
});
window.addEventListener("beforeunload", () => targetPoseEditor?.dispose(), { once: true });

elements.jointButtons.forEach((button, index) => {
  button.addEventListener("click", () => selectJoint(index));
});
elements.nudgeButtons.forEach((button) => {
  button.addEventListener("click", () => {
    setPreviewTarget(previewAngles[selectedJoint] + Number(button.dataset.delta));
  });
});
elements.targetInput.addEventListener("change", () => {
  setPreviewTarget(elements.targetInput.valueAsNumber);
});
elements.previewResetButton.addEventListener("click", () => resetPreview());
elements.syncPreviewButton.addEventListener("click", () => resetPreview());

elements.controlModeButtons.forEach((button) => {
  button.addEventListener("click", () => {
    setControlMode(button.dataset.mode);
  });
});

elements.speedInput.addEventListener("input", () => {
  syncMotionSpeedUi();
  updateCartesianInspector();
  setShortcutStatus(`机械臂速度已设为 ${getMotionSpeedPercent().toFixed(0)}%。`, "ready");
});
elements.speedInput.addEventListener("change", () => {
  if (previewKind !== "cartesian" || !cartesianPreview || !validCartesianDelta()) return;
  resetPreview({ quiet: true });
  requestCartesianPreview();
});

elements.pathModeButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const mode = button.dataset.pathMode;
    if (mode !== "smooth" && mode !== "linear") return;
    const replan = previewKind === "cartesian"
      && Boolean(cartesianPreview)
      && validCartesianDelta();
    activeXyzPathMode = mode;
    elements.pathModes.dataset.selected = mode;
    elements.pathModeButtons.forEach((item) => {
      const selected = item.dataset.pathMode === mode;
      item.classList.toggle("selected", selected);
      item.setAttribute("aria-pressed", selected ? "true" : "false");
    });
    setXyzStatus(
      mode === "smooth"
        ? "平滑曲线：关节空间 Ruckig 限跃度执行，末端路径允许自然弯曲。"
        : "末端直线：严格保持笛卡尔直线路径，启停仍使用平滑时间律。",
      "ready",
    );
    if (replan) {
      resetPreview({ quiet: true });
      requestCartesianPreview();
    }
  });
});

elements.servoEnabled.addEventListener("change", () => {
  updateCartesianInspector();
  setXyzStatus(
    elements.servoEnabled.checked
      ? "实时伺服已待命：按住平面键和鼠标拖动；自由阻尼状态会自动平滑启用关节位置控制。"
      : "实时伺服已关闭；拖动只生成预览。",
    elements.servoEnabled.checked ? "ready" : "",
  );
});

function invalidateCartesianPreview() {
  if (hasCartesianVisualPreview()) resetPreview({ quiet: true });
  updateCartesianInspector();
}

function nudgeCartesianAxis(axis, deltaMm) {
  const current = Object.fromEntries(
    ["x", "y", "z"].map((name) => [name, elements.xyzInputs[name].valueAsNumber]),
  );
  if (!["x", "y", "z"].includes(axis)
      || ![current.x, current.y, current.z, deltaMm].every(Number.isFinite)) {
    return false;
  }
  const next = current[axis] + deltaMm;
  elements.xyzInputs[axis].value = next.toFixed(1);
  syncTargetPoseFromPositionInputs();
  requestCartesianPreview();
  return true;
}

function syncTargetPoseFromPositionInputs() {
  const positionMm = targetPositionMmFromInputs();
  if (!positionMm.every(Number.isFinite)) {
    invalidateCartesianPreview();
    return false;
  }
  const actualPose = actualTargetPoseFromState();
  if (!targetPose) targetPose = cloneTargetPose(actualPose);
  if (!targetPose) return false;
  targetPose.position = positionMm.map((value) => value / 1000.0);
  targetPoseEditor?.setTargetPose(targetPose);
  updateTargetPoseReadout({ writePosition: false });
  setTargetGhostAppearance("pending");
  invalidateCartesianPreview();
  return true;
}

Object.values(elements.xyzInputs).forEach((input) => {
  input.addEventListener("input", syncTargetPoseFromPositionInputs);
});

elements.xyzQuickButtons.forEach((button) => {
  button.addEventListener("click", () => {
    nudgeCartesianAxis(button.dataset.axis, Number(button.dataset.delta));
  });
});

elements.xyzClearButton.addEventListener("click", () => {
  const actualPose = actualTargetPoseFromState();
  if (!actualPose) return;
  targetPoseEditor?.setRollRadians(0);
  applyEditedTargetPose(actualPose, "current");
});

elements.targetRoll.addEventListener("change", () => {
  if (!targetPoseEditor || !Number.isFinite(elements.targetRoll.valueAsNumber)) return;
  const requested = THREE.MathUtils.degToRad(elements.targetRoll.valueAsNumber);
  targetPoseEditor.adjustRoll(requested - targetPoseEditor.rollRadians, "roll-input");
  if (targetPoseEditor.state !== TARGET_EDIT_STATE.EDIT_ORIENTATION) {
    setXyzStatus("Roll 已更新；点击预览验证后再执行。", "ready");
  }
});

elements.gripperWidth.addEventListener("input", () => {
  if (!targetPose || !Number.isFinite(elements.gripperWidth.valueAsNumber)) return;
  targetGripperWidthMm = elements.gripperWidth.valueAsNumber;
  targetPose.gripperWidth = targetGripperWidthMm / 1000.0;
  targetPoseEditor?.setGripperWidth(targetPose.gripperWidth);
});

function requestCartesianPreview({
  fallbackDelta = null,
  dragGeneration = null,
  dragSource = null,
} = {}) {
  if (!motionAvailable() || !validCartesianDelta()) return null;
  if (previewDirty) resetPreview({ quiet: true });
  const delta = requestedCartesianDelta();
  const requestId = sendCommand("preview_xyz", delta);
  if (requestId !== null) {
    formalPreviewFallbackDelta = Array.isArray(fallbackDelta)
      ? fallbackDelta.map(Number)
      : null;
    formalPreviewDragGeneration = dragGeneration;
    formalPreviewDragSource = dragSource;
    formalPreviewTargetSignature = cartesianTargetSignature();
    commandPending = true;
    setXyzStatus(`正在求解 IK · ${xyzPathModeLabel()}…`, "ready");
    updateCartesianInspector();
  } else {
    formalPreviewFallbackDelta = null;
    formalPreviewDragGeneration = null;
    formalPreviewDragSource = null;
    formalPreviewTargetSignature = null;
  }
  return requestId;
}

function executeCartesianPreview() {
  if (previewKind !== "cartesian" || !cartesianPreview || !motionAvailable()) return;
  const [x_mm, y_mm, z_mm] = cartesianPreview.delta_xyz_mm;
  const requestId = sendMotionCommand("move_xyz", {
    x_mm,
    y_mm,
    z_mm,
    path_mode: activeXyzPathMode,
    orientation_xyzw: targetPose?.orientation?.map(Number),
  });
  if (requestId !== null) {
    commandPending = true;
    setXyzStatus(`已提交${xyzPathModeLabel()}轨迹，等待机械臂执行。`, "ready");
    updateCartesianInspector();
  }
}

elements.xyzPreviewButton.addEventListener("click", requestCartesianPreview);
elements.xyzExecuteButton.addEventListener("click", executeCartesianPreview);

function executePreview() {
  if (previewKind !== "joint" || !previewDirty || !motionAvailable()) return;
  const target = previewAngles[selectedJoint];
  const requestId = sendMotionCommand("move_joint_target", {
    joint: selectedJoint + 1,
    target_deg: target,
  });
  if (requestId !== null) {
    commandPending = true;
    updateInspector();
    setPreviewStatus(`已提交 J${selectedJoint + 1} ${target.toFixed(1)}°，等待机械臂反馈。`, "ready");
  }
}

elements.executeButton.addEventListener("click", executePreview);

function resetView() {
  cameraCalibrationViewActive = false;
  restoreSceneCameraProjection();
  calibrationCamera.helper.visible = true;
  camera.position.copy(DEFAULT_CAMERA_POSITION);
  controls.target.copy(DEFAULT_CAMERA_TARGET);
  controls.update();
}

function setSimulationReloadButton(label, { disabled = false, title = null } = {}) {
  if (elements.resetViewButtonLabel) elements.resetViewButtonLabel.textContent = label;
  elements.resetViewButton.disabled = disabled;
  if (title) elements.resetViewButton.title = title;
}

function markSceneResourcesForUpload() {
  scene.traverse((object) => {
    const geometry = object.geometry;
    if (geometry) {
      Object.values(geometry.attributes ?? {}).forEach((attribute) => {
        attribute.needsUpdate = true;
      });
      if (geometry.index) geometry.index.needsUpdate = true;
    }
    const materials = Array.isArray(object.material)
      ? object.material
      : object.material ? [object.material] : [];
    materials.forEach((material) => {
      material.needsUpdate = true;
      Object.values(material).forEach((value) => {
        if (value?.isTexture) value.needsUpdate = true;
      });
    });
  });
}

function resizeSimulationRenderer() {
  const rect = elements.canvas.parentElement?.getBoundingClientRect();
  if (!rect || rect.width <= 0 || rect.height <= 0) return;
  const width = Math.round(rect.width);
  const height = Math.round(rect.height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function finishSimulationReload() {
  window.clearTimeout(simulationRecoveryTimer);
  window.clearTimeout(simulationRestoreTimer);
  simulationRecoveryTimer = undefined;
  simulationRestoreTimer = undefined;
  try {
    rebuildSceneEnvironment();
    markSceneResourcesForUpload();
    resizeSimulationRenderer();
    simulationRendererAvailable = true;
    simulationReloading = false;
    setSimulationReloadButton("重载仿真", {
      title: "重新初始化 3D 仿真渲染器，不刷新控制页面",
    });
  } catch (error) {
    console.error("3D simulation recovery failed", error);
    simulationRendererAvailable = false;
    simulationReloading = false;
    setSimulationReloadButton("恢复仿真", {
      title: `3D 仿真恢复失败：${error.message}`,
    });
  }
}

function markSimulationReloadFailed() {
  if (!simulationReloading) return;
  simulationReloading = false;
  setSimulationReloadButton("恢复仿真", {
    title: "WebGL 恢复超时；再次点击重试。控制连接未受影响。",
  });
}

function reloadSimulation() {
  if (simulationReloading) return;
  simulationReloading = true;
  resetView();
  setSimulationReloadButton("重载中…", {
    disabled: true,
    title: "正在重新初始化 3D 仿真；机械臂控制连接保持不变。",
  });
  const context = renderer.getContext();
  const contextLost = context.isContextLost();
  const lossExtension = context.getExtension("WEBGL_lose_context");
  if (!contextLost && !lossExtension) {
    finishSimulationReload();
    return;
  }
  simulationRendererAvailable = false;
  if (contextLost) {
    renderer.forceContextRestore();
  } else {
    renderer.forceContextLoss();
    simulationRestoreTimer = window.setTimeout(() => {
      renderer.forceContextRestore();
    }, 80);
  }
  simulationRecoveryTimer = window.setTimeout(markSimulationReloadFailed, 1600);
}

elements.canvas.addEventListener("webglcontextlost", (event) => {
  event.preventDefault();
  simulationRendererAvailable = false;
  if (!simulationReloading) {
    setSimulationReloadButton("恢复仿真", {
      title: "WebGL 上下文已丢失；点击重新初始化仿真。",
    });
  }
});
elements.canvas.addEventListener("webglcontextrestored", finishSimulationReload);
elements.resetViewButton.addEventListener("click", reloadSimulation);

elements.tableCalibrationButton?.addEventListener("click", () => {
  const next = !tableCalibrationEditing;
  if (next) {
    setCameraCalibrationEditing(false);
    setLightCalibrationEditing(false);
  }
  setTableCalibrationEditing(next);
});
elements.tableCalibrationClose?.addEventListener("click", () => {
  setTableCalibrationEditing(false);
  elements.canvas.focus({ preventScroll: true });
});
elements.tableFocusButton?.addEventListener("click", focusTabletop);
elements.tableAutoAlignButton?.addEventListener("click", () => {
  elements.tableAutoAlignButton.disabled = true;
  setTableAutoAlignStatus("正在应用独立桌边标注…");
  window.dispatchEvent(new Event("arx5-table-auto-align-request"));
});
elements.tableCalibrationReset?.addEventListener("click", () => {
  applyTableCalibration(DEFAULT_TABLE_CALIBRATION, { syncInputs: true });
  focusTabletop();
});
Object.entries(elements.tableInputs).forEach(([key, input]) => {
  if (!input) return;
  input.addEventListener("input", () => {
    if (!Number.isFinite(input.valueAsNumber)) return;
    applyTableCalibration({
      ...tableCalibrationState,
      [key]: input.valueAsNumber,
    });
  });
  input.addEventListener("change", () => {
    if (Number.isFinite(input.valueAsNumber)) {
      applyTableCalibration({
        ...tableCalibrationState,
        [key]: input.valueAsNumber,
      });
    }
    syncTableCalibrationInputs();
  });
});

elements.cameraCalibrationButton?.addEventListener("click", () => {
  const next = !cameraCalibrationEditing;
  if (next) {
    setTableCalibrationEditing(false);
    setLightCalibrationEditing(false);
  }
  setCameraCalibrationEditing(next);
});
elements.cameraCalibrationClose?.addEventListener("click", () => {
  setCameraCalibrationEditing(false);
  elements.canvas.focus({ preventScroll: true });
});
elements.cameraFocusButton?.addEventListener("click", focusCalibrationCamera);
elements.cameraViewButton?.addEventListener("click", viewFromCalibrationCamera);
elements.cameraCalibrationReset?.addEventListener("click", () => {
  applyCameraCalibration(DEFAULT_CAMERA_CALIBRATION, { syncInputs: true });
  focusCalibrationCamera();
});
Object.entries(elements.cameraInputs).forEach(([key, input]) => {
  if (!input) return;
  input.addEventListener("input", () => {
    if (!Number.isFinite(input.valueAsNumber)) return;
    applyCameraCalibration({
      ...cameraCalibrationState,
      [key]: input.valueAsNumber,
    });
  });
  input.addEventListener("change", () => {
    if (Number.isFinite(input.valueAsNumber)) {
      applyCameraCalibration({
        ...cameraCalibrationState,
        [key]: input.valueAsNumber,
      });
    }
    syncCameraCalibrationInputs();
  });
});

elements.lightCalibrationButton?.addEventListener("click", () => {
  const next = !lightCalibrationEditing;
  if (next) {
    setTableCalibrationEditing(false);
    setCameraCalibrationEditing(false);
  }
  setLightCalibrationEditing(next);
});
elements.lightCalibrationClose?.addEventListener("click", () => {
  setLightCalibrationEditing(false);
  elements.canvas.focus({ preventScroll: true });
});
elements.lightCalibrationReset?.addEventListener("click", () => {
  applyLightCalibration(DEFAULT_LIGHT_CALIBRATION, { syncInputs: true });
});
Object.entries(elements.lightInputs).forEach(([key, input]) => {
  if (!input) return;
  input.addEventListener("input", () => {
    if (!Number.isFinite(input.valueAsNumber)) return;
    applyLightCalibration({
      ...lightCalibrationState,
      [key]: input.valueAsNumber,
    });
  });
  input.addEventListener("change", () => {
    if (Number.isFinite(input.valueAsNumber)) {
      applyLightCalibration({
        ...lightCalibrationState,
        [key]: input.valueAsNumber,
      });
    }
    syncLightCalibrationInputs();
  });
});

window.addEventListener("arx5-state", (event) => handleArmState(event.detail));
window.addEventListener("arx5-table-edge-guides-change", (event) => {
  setTableAutoAlignStatus(
    event.detail?.active ? "桌边已保存，正在对齐 3D 桌面…" : "独立桌边标注已清除",
    event.detail?.active ? "idle" : "error",
  );
});
window.addEventListener("arx5-table-auto-align-result", (event) => {
  const detail = event.detail ?? {};
  if (elements.tableAutoAlignButton) elements.tableAutoAlignButton.disabled = false;
  if (!detail.ok || !detail.calibration) {
    setTableAutoAlignStatus(detail.error || "未能识别到稳定桌边", "error");
    return;
  }
  if (!detail.applied) {
    applyTableCalibration({
      ...tableCalibrationState,
      ...detail.calibration,
    }, { syncInputs: true });
  }
  const [xMm, yMm] = [detail.calibration.xMm, detail.calibration.yMm]
    .map((value) => Number(value).toFixed(0));
  const yawDeg = Number(detail.calibration.yawDeg).toFixed(1);
  setTableAutoAlignStatus(
    `${detail.cached ? "已应用缓存" : "已对齐"} X ${xMm} · Y ${yMm} · Yaw ${yawDeg}°`,
    "success",
  );
});
window.addEventListener("arx5-cube-work-area", (event) => handleCubeWorkArea(event.detail));
window.addEventListener("arx5-table-edge-guides-preview", (event) => {
  handleTableEdgeGuidePreview(event.detail);
});
window.addEventListener("arx5-motion-command-start", (event) => {
  const { action, params = {} } = event.detail ?? {};
  if (action === "cube_pick_execute") {
    cubeMotionAction = action;
    carriedCubeId = selectedDetectedCubeId;
    carriedCubeAttached = false;
    carriedCubeReleasedFromGripper = false;
    carriedCubeHoldUntilMs = Infinity;
  } else if (["cube_move_execute", "cube_place_execute"].includes(action)) {
    cubeMotionAction = action;
  } else if (action === "cube_stack_execute") {
    cubeMotionAction = action;
  }
  if (MOTION_EXECUTION_ACTIONS.includes(action)) {
    beginMotionFeedbackFollow();
    beginPlannedPathExecution();
  }
  if (action === "move_xyz") {
    if (!plannedPathCurve) {
      const start = finitePose(armState.eef_pose_6d)
        ? armState.eef_pose_6d.slice(0, 3).map((value) => Number(value) * 1000)
        : null;
      const delta = [params.x_mm, params.y_mm, params.z_mm].map(Number);
      if (start?.every(Number.isFinite) && delta.every(Number.isFinite)) {
        setPlannedPath(
          [start, start.map((value, index) => value + delta[index])],
          params.path_mode === "linear" ? "linear" : "smooth",
        );
        beginPlannedPathExecution();
      }
    }
    return;
  }
  if (!["move_joint", "move_joint_target", "reset_joint"].includes(action)
      || !finiteJointVector(actualAngles)) return;
  const jointIndex = Number(params.joint) - 1;
  if (!Number.isInteger(jointIndex) || jointIndex < 0 || jointIndex >= actualAngles.length) return;
  const targetAngles = actualAngles.slice();
  if (action === "move_joint") {
    targetAngles[jointIndex] += Number(params.delta_deg);
  } else if (action === "move_joint_target") {
    targetAngles[jointIndex] = Number(params.target_deg);
  } else {
    const reference = armState.reference_joint_position_deg;
    if (!finiteJointVector(reference)) return;
    targetAngles[jointIndex] = Number(reference[jointIndex]);
  }
  if (!finiteJointVector(targetAngles)) return;
  const path = jointEndEffectorPathMm(actualAngles, targetAngles);
  setPlannedPath(path, "joint");
  beginPlannedPathExecution();
  if (Array.isArray(path) && path.length >= 2) {
    const start = new THREE.Vector3(...path[0].map((value) => value / 1000));
    const target = new THREE.Vector3(...path.at(-1).map((value) => value / 1000));
    setCartesianGuide(start, target, 0xffc06a, false);
  }
});
window.addEventListener("arx5-cube-detection", (event) => handleCubeDetection(event.detail));
window.addEventListener("arx5-cube-pick-state", (event) => {
  if (event.detail?.phase === "reset") {
    cubePickVisualPinned = false;
    cubePickVisualHoldUntilMs = -Infinity;
    setCubePickGraspGhost(null);
    if (plannedPathMode?.startsWith("cube-")) setPlannedPath(null);
    return;
  }
  if (!["planning", "executing"].includes(event.detail?.phase)) return;
  if (event.detail?.phase === "executing") beginPlannedPathExecution();
  cubePickVisualPinned = true;
  cubePickVisualHoldUntilMs = -Infinity;
  const selectedCube = selectedDetectedCube();
  if (selectedCube) selectedCube.targetOpacity = Math.max(selectedCube.targetOpacity, 1.0);
});
window.addEventListener("arx5-park-execution-start", () => {
  parkControlFollowing = true;
  const actualPose = actualTargetPoseFromState();
  if (actualPose) targetPoseEditor?.beginFeedbackFollow(actualPose);
  updateCartesianInspector();
});
window.addEventListener("arx5-park-execution-cancel", () => {
  parkControlFollowing = false;
  targetPoseEditor?.endFeedbackFollow();
  updateCartesianInspector();
});
window.addEventListener("arx5-park-preview-start", () => {
  commandPending = true;
  setShortcutStatus("正在生成回参考 Ghost 与末端轨迹…", "ready");
  updateInspector();
  updateCartesianInspector();
});
window.addEventListener("arx5-park-preview-reset", () => {
  if (previewKind === "park") resetPreview({ quiet: true });
});
window.addEventListener("arx5-teleop-state", (event) => {
  const message = event.detail ?? {};
  const serverPreview = message.preview;
  const finitePreview = finiteJointVector(
    serverPreview?.target_joint_position_deg,
  );
  if (message.action === "update" && finitePreview) {
    if (!hasTeleoperationPreview()) displayedPreviewAngles = actualAngles.slice();
    previewKind = serverPreview.reachable === false
      ? "teleoperation-limit"
      : "teleoperation";
    previewDirty = true;
    cartesianPreview = null;
    previewAngles = serverPreview.target_joint_position_deg.map(Number);
    if (serverPreview.gripper_state === "open") {
      const openWidth = Number(
        document.querySelector("#teleoperationOpenWidth")?.value,
      );
      if (Number.isFinite(openWidth)) targetGripperWidthMm = openWidth;
    }
    displayedTargetGripperWidthMm = targetGripperWidthMm;
    setModelGripperWidth(previewModel, targetGripperWidthMm);
    updatePreviewVisibility();
    updateInspector();
    if (elements.handState && serverPreview.reachable === false) {
      elements.handState.textContent = serverPreview.error ?? "目标已到安全边界";
    }
    return;
  }
  if ((message.action === "stop" || !message.ok) && hasTeleoperationPreview()) {
    targetGripperWidthMm = actualGripperWidthMm;
    displayedTargetGripperWidthMm = actualGripperWidthMm;
    resetPreview({ quiet: true });
    setModelGripperWidth(previewModel, displayedGripperWidthMm);
  }
});
window.addEventListener("arx5-servo-state", (event) => {
  const message = event.detail;
  if (message.kind === "gripper"
      && (!message.ok || message.action === "stop")) {
    stopGripperKeyControl({ notifyServer: false });
  }
  if (message.kind === "gripper") return;
  const servoPreview = message.preview;
  if (message.ok && message.action === "update" && cartesianDrag?.liveServo
      && finiteJointVector(servoPreview?.target_joint_position_deg)) {
    const reachable = servoPreview.reachable !== false;
    previewKind = reachable ? "cartesian-servo" : "cartesian-servo-limit";
    previewDirty = true;
    cartesianPreview = null;
    previewAngles = servoPreview.target_joint_position_deg.map(Number);
    if (reachable && Array.isArray(servoPreview.delta_xyz_mm)
        && servoPreview.delta_xyz_mm.length === 3
        && servoPreview.delta_xyz_mm.every(Number.isFinite)) {
      cartesianDrag.lastValidDelta = servoPreview.delta_xyz_mm.map(Number);
    }
    const guideColor = reachable ? cartesianDrag.plane.color : LIMIT_PREVIEW_COLOR;
    if (cartesianTargetMarker && cartesianGuideLine) {
      cartesianTargetMarker.material.color.setHex(guideColor);
      cartesianGuideLine.material.color.setHex(guideColor);
    }
    updatePreviewVisibility();
    updateInspector();
    updateCartesianInspector();
    if (!reachable) {
      setXyzStatus(
        `已到达当前可达边界；目标保持在红色极限姿态。${servoPreview.error ?? "请往回拖动。"}`,
        "error",
      );
    }
  }
  if (!message.ok && cartesianDrag?.liveServo) {
    cancelCartesianDrag(message.error ?? "实时伺服已停止。", false);
    setXyzStatus(message.error ?? "实时伺服已停止。", "error");
  }
});
window.addEventListener("arx5-command-result", (event) => {
  const message = event.detail;
  if (MOTION_EXECUTION_ACTIONS.includes(message.action)) {
    finishPlannedPathExecution(message.ok === true);
    endMotionFeedbackFollow();
  }
  if (message.action === "cube_pick_preview") {
    if (message.ok && message.result?.kind === "cube_pick_preview") {
      setPlannedPath(message.result.path_xyz_mm, "cube-pick");
      setCubePickGraspGhost(message.result);
      cubePickVisualPinned = true;
      const graspLabel = message.result.grasp_mode === "side"
        ? `${Number(message.result.grasp_tilt_deg) || 45}°斜抓`
        : "俯抓";
      setShortcutStatus(`${graspLabel}路径已通过连续 IK 预检；再次点击抓取按钮执行。`, "ready");
    } else {
      setPlannedPath(null);
      setCubePickGraspGhost(null);
      cubePickVisualPinned = false;
      setShortcutStatus(message.error ?? "无法生成抓取路径。", "error");
    }
  } else if (["cube_move_preview", "cube_place_preview", "cube_stack_preview"].includes(message.action)) {
    if (message.ok && Array.isArray(message.result?.path_xyz_mm)) {
      setPlannedPath(
        message.result.path_xyz_mm,
        message.action === "cube_move_preview"
          ? "cube-move"
          : message.action === "cube_stack_preview" ? "cube-stack" : "cube-place",
      );
      if (message.action !== "cube_stack_preview") setCubeStageTcpGhost(message.result);
      setShortcutStatus(
        message.action === "cube_move_preview"
          ? "持物移动轨迹已通过连续 IK 预检；再次点击移动按钮执行。"
          : message.action === "cube_stack_preview"
          ? "完整抓取、搬运、放置与回程已统一预检；确认后连续执行。"
          : "堆叠下降、松爪和撤离轨迹已通过预检；再次点击放下按钮执行。",
        "ready",
      );
    } else {
      setPlannedPath(null);
      setCubeStageTcpGhost(null);
      setShortcutStatus(message.error ?? "无法生成当前阶段轨迹。", "error");
    }
  } else if (["cube_pick_execute", "cube_move_execute", "cube_place_execute", "cube_stack_execute"].includes(message.action)) {
    if (message.action === "cube_pick_execute" && message.ok) {
      attachCarriedCube({ force: true });
    } else if (message.action === "cube_pick_execute") {
      carriedCubeHoldUntilMs = performance.now() + 3000;
    } else if (message.action === "cube_place_execute" && message.ok) {
      detachCarriedCube();
    } else if (message.action === "cube_stack_execute") {
      detachCarriedCube();
    }
    cubeMotionAction = null;
    cubePickVisualPinned = false;
    cubePickVisualHoldUntilMs = performance.now() + (message.ok ? 3000 : 0);
  } else if (message.action === "preview_park") {
    commandPending = false;
    const result = message.result;
    if (message.ok && result?.kind === "park_preview"
        && finiteJointVector(result.target_joint_position_deg)) {
      if (previewDirty) resetPreview({ quiet: true });
      previewKind = "park";
      previewDirty = true;
      cartesianPreview = null;
      previewAngles = result.target_joint_position_deg.map(Number);
      displayedPreviewAngles = actualAngles.slice();
      setModelGripperWidth(previewModel, actualGripperWidthMm);
      setPlannedPath(result.path_xyz_mm, "park");
      updatePreviewVisibility();
      updateInspector();
      updateCartesianInspector();
      setShortcutStatus(
        `回参考预览就绪：最大关节变化 ${result.max_joint_delta_deg.toFixed(1)}°，轨迹 ${result.trajectory_duration_s.toFixed(1)} s；再次点击顶部按钮执行。`,
        "ready",
      );
    } else {
      if (previewKind === "park") resetPreview({ quiet: true });
      setShortcutStatus(message.error ?? "无法生成回参考轨迹预览。", "error");
    }
  } else if (message.action === "preview_xyz_live") {
    if (message.id !== livePreviewRequestId) return;
    const responseGeneration = livePreviewDragGeneration;
    const responseSource = livePreviewDragSource;
    livePreviewInFlight = false;
    livePreviewRequestId = null;
    livePreviewDragGeneration = null;
    livePreviewDragSource = null;
    const appliesToCartesianDrag = responseSource === "cartesian-plane"
      && Boolean(cartesianDrag)
      && cartesianDrag.generation === responseGeneration;
    const appliesToTargetEditor = responseSource === "target-editor"
      && Boolean(targetEditorLivePreview)
      && targetEditorLivePreview.generation === responseGeneration;
    const appliesToCurrentDrag = appliesToCartesianDrag || appliesToTargetEditor;
    const result = message.result;
    const validLiveResult = message.ok && result?.kind === "cartesian_preview"
      && finiteJointVector(result.target_joint_position_deg)
      && Array.isArray(result.delta_xyz_mm)
      && result.delta_xyz_mm.length === 3
      && result.delta_xyz_mm.every(Number.isFinite);
    if (appliesToCurrentDrag) {
      if (validLiveResult) {
        const previewSession = appliesToCartesianDrag
          ? cartesianDrag
          : targetEditorLivePreview;
        if (previewSession && !previewSession.previewInitialized) {
          const baseline = finiteJointVector(previewSession.startAngles)
            ? previewSession.startAngles
            : actualAngles;
          if (finiteJointVector(baseline)) displayedPreviewAngles = baseline.slice();
          previewSession.previewInitialized = true;
        }
        previewKind = "cartesian-live";
        previewDirty = true;
        cartesianPreview = null;
        previewAngles = result.target_joint_position_deg.map(Number);
        if (appliesToCartesianDrag) {
          cartesianDrag.lastValidDelta = result.delta_xyz_mm.map(Number);
          const color = cartesianDrag.plane.color;
          if (cartesianTargetMarker && cartesianGuideLine && color) {
            cartesianTargetMarker.material.color.setHex(color);
            cartesianGuideLine.material.color.setHex(color);
          }
        } else {
          targetEditorLivePreview.lastValidDelta = result.delta_xyz_mm.map(Number);
          targetPoseEditor?.setValidationState("valid");
          setTargetGhostAppearance("valid");
          setXyzStatus("实时 IK 可行；半透明机械臂正在连续跟随目标。", "ready");
        }
        updatePreviewVisibility();
        updateInspector();
        updateCartesianInspector();
      } else {
        if (previewDirty) previewKind = "cartesian-limit";
        if (appliesToCartesianDrag && cartesianTargetMarker && cartesianGuideLine) {
          cartesianTargetMarker.material.color.setHex(LIMIT_PREVIEW_COLOR);
          cartesianGuideLine.material.color.setHex(LIMIT_PREVIEW_COLOR);
        } else if (appliesToTargetEditor) {
          targetPoseEditor?.setValidationState("invalid");
          setTargetGhostAppearance("invalid");
        }
        updatePreviewVisibility();
        updateCartesianInspector();
        setXyzStatus(
          `已到达当前可达边界；半透明姿态保持在最后可达点。${message.error ?? "请往回拖动或换一个方向。"}`,
          "error",
        );
      }
    } else if (validLiveResult
        && responseGeneration === formalPreviewDragGeneration
        && responseSource === formalPreviewDragSource) {
      // The mouse may have been released while the newest live IK was still
      // in flight. Keep that result as the best fallback for formal preview.
      formalPreviewFallbackDelta = result.delta_xyz_mm.map(Number);
    }
    if (livePreviewQueued && activeReadOnlyPreviewDrag()) scheduleLiveCartesianPreview();
  } else if (message.action === "preview_xyz") {
    commandPending = false;
    const previewMatchesTarget = formalPreviewTargetSignature === cartesianTargetSignature();
    formalPreviewTargetSignature = null;
    if (!previewMatchesTarget) {
      formalPreviewFallbackDelta = null;
      formalPreviewDragGeneration = null;
      formalPreviewDragSource = null;
      targetPoseEditor?.setValidationState("pending");
      setTargetGhostAppearance("pending");
      updateCartesianInspector();
      setXyzStatus("目标在 IK 计算期间已改变，已丢弃过期预览。", "ready");
      if (targetPoseEditor?.state === TARGET_EDIT_STATE.EDIT_POSITION
          && motionAvailable() && validCartesianDelta()) {
        requestCartesianPreview();
      }
      return;
    }
    const result = message.result;
    if (message.ok && result?.kind === "cartesian_preview"
        && finiteJointVector(result.target_joint_position_deg)) {
      formalPreviewFallbackDelta = null;
      formalPreviewDragGeneration = null;
      formalPreviewDragSource = null;
      cartesianPreview = result;
      previewKind = "cartesian";
      previewDirty = true;
      previewAngles = result.target_joint_position_deg.map(Number);
      targetPoseEditor?.setValidationState("valid");
      setTargetGhostAppearance("valid");
      setPlannedPath(result.path_xyz_mm, result.path_mode);
      if (Array.isArray(result.start_xyz_mm) && Array.isArray(result.target_xyz_mm)) {
        const pathColor = result.path_mode === "linear" ? 0x76c9ff : 0xffc06a;
        setCartesianGuide(
          new THREE.Vector3(...result.start_xyz_mm.map((value) => value / 1000.0)),
          new THREE.Vector3(...result.target_xyz_mm.map((value) => value / 1000.0)),
          pathColor,
          false,
        );
      }
      updatePreviewVisibility();
      updateInspector();
      updateCartesianInspector();
      setXyzStatus(
        `IK 可行 · ${xyzPathModeLabel()}：最大关节变化 ${result.max_joint_delta_deg.toFixed(1)}°，位置 ${result.ik_position_error_mm.toFixed(2)} mm，姿态 ${result.ik_orientation_error_deg.toFixed(2)}°。确认后再执行。`,
        "ready",
      );
    } else {
      const fallback = formalPreviewFallbackDelta;
      formalPreviewFallbackDelta = null;
      formalPreviewDragGeneration = null;
      formalPreviewDragSource = null;
      resetPreview({ quiet: true });
      targetPoseEditor?.setValidationState("invalid");
      setTargetGhostAppearance("invalid");
      const current = requestedCartesianDelta();
      const canUseFallback = Array.isArray(fallback)
        && fallback.length === 3
        && fallback.every(Number.isFinite)
        && validCartesianDelta({
          x_mm: fallback[0],
          y_mm: fallback[1],
          z_mm: fallback[2],
        })
        && Math.hypot(
          fallback[0] - current.x_mm,
          fallback[1] - current.y_mm,
          fallback[2] - current.z_mm,
        ) >= 0.1;
      if (canUseFallback) {
        const currentMm = armState.eef_pose_6d.slice(0, 3)
          .map((value) => Number(value) * 1000.0);
        elements.xyzInputs.x.value = (currentMm[0] + fallback[0]).toFixed(1);
        elements.xyzInputs.y.value = (currentMm[1] + fallback[1]).toFixed(1);
        elements.xyzInputs.z.value = (currentMm[2] + fallback[2]).toFixed(1);
        syncTargetPoseFromPositionInputs();
        if (requestCartesianPreview() !== null) {
          setXyzStatus("已到达当前姿态的 IK 边界，正在回收到最近可达点复核…", "ready");
        } else {
          setXyzStatus("最近可达点已经失效，请重新拖动。", "error");
        }
      } else {
        setXyzStatus(
          message.error ?? "XYZ 目标无法求解。请减小位移或换一个方向。",
          "error",
        );
      }
    }
  } else if (message.action === "park") {
    commandPending = false;
    parkControlFollowing = false;
    targetPoseEditor?.endFeedbackFollow();
    setCubeStageTcpGhost(null);
    if (previewKind === "park") resetPreview({ quiet: true, preservePath: message.ok });
    const actualPose = actualTargetPoseFromState();
    if (actualPose) {
      targetPoseEditor?.setRollRadians(0);
      applyEditedTargetPose(actualPose, "park-complete");
    }
    setShortcutStatus(
      message.ok
        ? "机械臂与夹爪已回到参考状态，并主动保持。"
        : (message.error ?? "回参考失败。"),
      message.ok ? "ready" : "error",
    );
  } else if (message.action === "move_xyz") {
    commandPending = false;
    if (message.ok) {
      const finalError = message.result?.final_position_error_mm;
      resetPreview({ quiet: true, preservePath: true });
      const actualPose = actualTargetPoseFromState();
      if (actualPose) {
        targetPoseEditor?.setRollRadians(0);
        applyEditedTargetPose(actualPose, "motion-complete");
      }
      setXyzStatus(
        Number.isFinite(finalError)
          ? `XYZ 移动完成，末端位置误差 ${finalError.toFixed(2)} mm。`
          : "XYZ 移动完成。",
        "ready",
      );
    } else {
      resetPreview({ quiet: true });
      setXyzStatus(message.error ?? "XYZ 移动失败。", "error");
    }
  } else if (["move_joint", "move_joint_target", "reset_joint"].includes(message.action)) {
    commandPending = false;
    if (message.ok) resetPreview({ quiet: true, preservePath: true });
    else {
      updateInspector();
      setPreviewStatus(message.error ?? "关节运动失败。", "error");
    }
  } else if (message.action === "activate" && !message.ok && commandPending) {
    commandPending = false;
    updateInspector();
    updateCartesianInspector();
    setPreviewStatus(message.error ?? "无法启用关节位置控制。", "error");
    setXyzStatus(message.error ?? "无法启用关节位置控制。", "error");
  }
});

const resizeObserver = new ResizeObserver((entries) => {
  const rect = entries[0]?.contentRect;
  if (!rect || rect.width <= 0 || rect.height <= 0) return;
  const width = Math.round(rect.width);
  const height = Math.round(rect.height);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
});
resizeObserver.observe(elements.canvas.parentElement);

function render(now) {
  const elapsedSeconds = Math.min(0.1, Math.max(0, (now - lastRenderTime) / 1000));
  lastRenderTime = now;
  if (!simulationRendererAvailable) {
    requestAnimationFrame(render);
    return;
  }
  if (plannedPathExecutionActive && plannedPathCurve && plannedPathFeedbackDirty) {
    plannedPathFeedbackDirty = false;
    const current = feedbackEefPosition();
    plannedPathExecutionProgress = Math.max(
      plannedPathExecutionProgress,
      nearestPlannedPathProgress(current),
    );
    updateExecutedFeedbackPath();
    updatePlannedPathSplit();
  }
  if (plannedPathFlowLine?.visible && plannedPathFlowCurve
      && now - lastPlannedPathFlowUpdateAt >= PATH_FLOW_UPDATE_INTERVAL_MS) {
    lastPlannedPathFlowUpdateAt = now;
    const progress = (now % 3200) / 3200;
    const positions = plannedPathFlowLine.geometry.attributes.position;
    const tailLength = 0.1;
    for (let index = 0; index < positions.count; index += 1) {
      const amount = index / Math.max(1, positions.count - 1);
      const localProgress = Math.max(0, progress - tailLength + amount * tailLength);
      const pathProgress = plannedPathFlowStartProgress
        + (1 - plannedPathFlowStartProgress) * localProgress;
      const point = plannedPathFlowCurve.getPointAt(pathProgress);
      positions.setXYZ(index, point.x, point.y, point.z);
    }
    positions.needsUpdate = true;
  }
  const measuredGripperWidth = Number(armState.gripper_position_mm);
  if (cubeMotionAction === "cube_pick_execute"
      && !carriedCubeAttached
      && Number.isFinite(measuredGripperWidth)
      && measuredGripperWidth <= 50) {
    attachCarriedCube();
  } else if (cubeMotionAction === "cube_place_execute"
      && carriedCubeAttached
      && Number.isFinite(measuredGripperWidth)
      && measuredGripperWidth >= 60) {
    detachCarriedCube();
  }
  updateCarriedCubePose();
  for (const [candidateId, cube] of detectedCubes) {
    const held = candidateId === selectedDetectedCubeId
      && (cubePickVisualPinned || now < cubePickVisualHoldUntilMs);
    const carried = candidateId === carriedCubeId
      && (carriedCubeAttached || now < carriedCubeHoldUntilMs);
    if (now - cube.lastUpdateMs > 1800 && !held && !carried) cube.targetOpacity = 0;
    if (!cube.root.visible) continue;
    const cubeFollow = 1 - Math.exp(-11 * elapsedSeconds);
    cube.root.position.lerp(cube.targetPosition, cubeFollow);
    cube.root.quaternion.slerp(cube.targetQuaternion, cubeFollow);
    cube.material.opacity += (cube.targetOpacity - cube.material.opacity) * cubeFollow;
    cube.outlineMaterial.opacity = Math.min(0.82, cube.material.opacity * 0.9);
    cube.selectedOutlineMaterial.opacity = Math.min(0.98, cube.material.opacity);
    cube.candidateOutlineMaterial.opacity = Math.min(0.62, cube.material.opacity * 2.4);
    cube.headingMaterial.opacity = Math.min(0.9, cube.material.opacity);
    if (cube.targetOpacity === 0 && cube.material.opacity < 0.025) {
      cube.root.visible = false;
    }
  }
  updateLoadingSkeleton(elapsedSeconds);
  if (actualModel) {
    const feedbackFollowRate = armState.busy ? 55 : 28;
    const follow = 1 - Math.exp(-feedbackFollowRate * elapsedSeconds);
    let changed = false;
    displayedAngles = displayedAngles.map((value, index) => {
      const next = value + (actualAngles[index] - value) * follow;
      if (Math.abs(next - value) > 1e-4) changed = true;
      return next;
    });
    if (changed) setModelAngles(actualModel, displayedAngles);

    const gripperFollowRate = armState.busy ? 65 : 34;
    const gripperFollow = 1 - Math.exp(-gripperFollowRate * elapsedSeconds);
    const nextGripperWidth = displayedGripperWidthMm
      + (actualGripperWidthMm - displayedGripperWidthMm) * gripperFollow;
    if (Math.abs(nextGripperWidth - displayedGripperWidthMm) > 1e-4) {
      displayedGripperWidthMm = nextGripperWidth;
      setModelGripperWidth(actualModel, displayedGripperWidthMm);
      if (!hasTeleoperationPreview()) {
        setModelGripperWidth(previewModel, displayedGripperWidthMm);
      }
    }
    const targetGripperFollow = 1 - Math.exp(-32 * elapsedSeconds);
    const nextTargetGripperWidth = displayedTargetGripperWidthMm
      + (targetGripperWidthMm - displayedTargetGripperWidthMm) * targetGripperFollow;
    if (Math.abs(nextTargetGripperWidth - displayedTargetGripperWidthMm) > 1e-3) {
      displayedTargetGripperWidthMm = nextTargetGripperWidth;
      setGripperVisualWidth(
        targetGripperGhost?.gripper,
        displayedTargetGripperWidthMm,
        Number.isFinite(armState.gripper_max_mm) ? armState.gripper_max_mm : 60,
      );
    }
  }
  if (previewModel && previewDirty) {
    const follow = 1 - Math.exp(-26 * elapsedSeconds);
    let changed = false;
    displayedPreviewAngles = displayedPreviewAngles.map((value, index) => {
      const next = value + (previewAngles[index] - value) * follow;
      if (Math.abs(next - value) > 1e-4) changed = true;
      return next;
    });
    if (changed) setModelAngles(previewModel, displayedPreviewAngles);
  }
  targetPoseEditor?.update(now);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}
requestAnimationFrame(render);

// The official meshes are ~13.4 MB on a cold cache and arrive as seven
// independent downloads. Instead of hiding the stage behind an opaque scrim,
// the low-poly `FALLBACK_LINKS` boxes are assembled immediately as a wireframe
// and each link brightens when its STL lands, so the wait shows the real
// kinematic chain and its own progress. Wireframe is deliberate: a solid
// placeholder could be mistaken for the loaded arm.
//
// This reuses the offline degraded path rather than adding one. `BoxGeometry`
// is indexed, so `createRobotModel` classifies link6 as non-dynamic and
// attaches the lightweight finger overlay exactly as it does when jsDelivr is
// unreachable.
const SKELETON_PENDING_OPACITY = 0.14;
const SKELETON_READY_OPACITY = 0.46;
const SKELETON_FADE_RATE = 9;

let loadingSkeleton = null;

function createLoadingSkeleton() {
  const model = createRobotModel(
    FALLBACK_LINKS.map((_, index) => fallbackGeometry(index)),
    false,
  );
  model.root.name = "loading-skeleton";

  // `meshes[k]` is built from `geometries[k]`, so a mesh's position in the list
  // is its download index. The finger overlay belongs to link6.
  const linkMeshes = model.meshes.map((mesh) => [mesh]);
  (model.gripper?.meshes ?? []).forEach((mesh) => linkMeshes[6]?.push(mesh));

  const linkMaterials = linkMeshes.map((meshes) => meshes.map((mesh) => {
    const replaced = mesh.material;
    mesh.material = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      wireframe: true,
      transparent: true,
      opacity: 0,
      depthWrite: false,
    });
    replaced.dispose();
    mesh.castShadow = false;
    mesh.receiveShadow = false;
    mesh.renderOrder = 4;
    return mesh.material;
  }));

  setModelAngles(model, displayedAngles);
  return {
    model,
    linkMaterials,
    targets: linkMeshes.map(() => SKELETON_PENDING_OPACITY),
  };
}

function markSkeletonLinkReady(index) {
  if (!loadingSkeleton) return;
  if (!Number.isInteger(index) || index < 0) return;
  if (index >= loadingSkeleton.targets.length) return;
  loadingSkeleton.targets[index] = SKELETON_READY_OPACITY;
}

function updateLoadingSkeleton(elapsedSeconds) {
  if (!loadingSkeleton) return;
  const follow = 1 - Math.exp(-SKELETON_FADE_RATE * elapsedSeconds);
  loadingSkeleton.linkMaterials.forEach((materials, index) => {
    const target = loadingSkeleton.targets[index];
    materials.forEach((material) => {
      material.opacity += (target - material.opacity) * follow;
    });
  });
}

function disposeLoadingSkeleton() {
  if (!loadingSkeleton) return;
  const { model } = loadingSkeleton;
  loadingSkeleton = null;
  scene.remove(model.root);
  // `createGripperVisual` shares one geometry across both fingers, so collect
  // through a set rather than disposing per mesh.
  const geometries = new Set();
  const materials = new Set();
  model.root.traverse((object) => {
    if (!object.isMesh) return;
    geometries.add(object.geometry);
    materials.add(object.material);
  });
  geometries.forEach((geometry) => geometry.dispose());
  materials.forEach((material) => material.dispose());
}

function hideLoadingOverlay() {
  const overlay = elements.loading;
  if (!overlay || overlay.hidden) return;
  const finish = () => {
    if (overlay.hidden) return;
    overlay.hidden = true;
    overlay.classList.remove("is-done");
  };
  // `hidden` resolves to `display: none`, which cancels transitions outright.
  // The fade has to run first and only then leave layout.
  if (window.matchMedia?.("(prefers-reduced-motion: reduce)").matches) {
    finish();
    return;
  }
  overlay.addEventListener("transitionend", finish, { once: true });
  // `transitionend` never arrives while the tab is backgrounded.
  window.setTimeout(finish, 600);
  overlay.classList.add("is-done");
}

async function initializeModel() {
  setPill("3D 模型加载中", "pending");
  loadingSkeleton = createLoadingSkeleton();
  try {
    const {
      geometries,
      degraded,
      d405Geometry,
      d405Degraded,
      cacheEnabled,
      cacheHits,
    } = await loadGeometries(markSkeletonLinkReady);
    disposeLoadingSkeleton();
    actualModel = createRobotModel(geometries, false, d405Geometry);
    previewModel = createRobotModel(geometries, true, d405Geometry);
    selectionRing = createSelectionRing();
    createCartesianIndicators();
    initializeTargetPoseEditing();
    modelsReady = true;
    setModelAngles(actualModel, displayedAngles);
    setModelAngles(previewModel, previewAngles);
    setModelGripperWidth(actualModel, displayedGripperWidthMm);
    setModelGripperWidth(previewModel, displayedGripperWidthMm);
    targetGripperWidthMm = Number.isFinite(targetPose?.gripperWidth)
      ? targetPose.gripperWidth * 1000.0
      : displayedGripperWidthMm;
    displayedTargetGripperWidthMm = targetGripperWidthMm;
    selectJoint(0);
    setControlMode("xyz");
    updatePreviewVisibility();
    hideLoadingOverlay();
    const anyDegraded = degraded || d405Degraded;
    const modelStatus = anyDegraded
      ? "3D 模型降级可用"
      : cacheHits === FALLBACK_LINKS.length
        ? "X5 + D405 模型已从缓存加载"
        : cacheEnabled
          ? "X5 + D405 官方模型已加载并缓存"
          : "X5 + D405 官方模型已加载";
    setPill(modelStatus, anyDegraded ? "warning" : "ok");
    handleArmState(getArmState());
  } catch (error) {
    disposeLoadingSkeleton();
    console.error(error);
    elements.loadingText.textContent = `3D 模型初始化失败：${error.message}`;
    setPill("3D 模型错误", "bad");
    setPreviewStatus("3D 模型无法初始化；高级滑块仍可使用。", "error");
  }
}

initializeModel();
syncMotionSpeedUi();
updateCartesianInputLimits();

window.addEventListener("arx5-handeye-board-pose", (event) => {
  setHandeyeBoardObservation(event.detail ?? null);
});

window.addEventListener("arx5-handeye-plan", (event) => {
  const plan = event.detail ?? {};
  const waypoints = Array.isArray(plan.waypoints) ? plan.waypoints : [];
  const validationWaypoints = Array.isArray(plan.validation_waypoints)
    ? plan.validation_waypoints
    : [];
  const targets = waypoints
    .map((waypoint) => Array.isArray(waypoint?.joint_position_rad)
      ? waypoint.joint_position_rad.map((value) => Number(value) * 180 / Math.PI)
      : null)
    .filter(finiteJointVector);
  if (!targets.length || !finiteJointVector(actualAngles)) return;
  const validationTargets = validationWaypoints
    .map((waypoint) => Array.isArray(waypoint?.joint_position_rad)
      ? waypoint.joint_position_rad.map((value) => Number(value) * 180 / Math.PI)
      : null)
    .filter(finiteJointVector);
  const path = [];
  handeyePlanArrows.clear();
  let previous = actualAngles.slice();
  let previousPoint = currentEefWorldPosition();
  for (const [index, target] of targets.entries()) {
    const segment = jointEndEffectorPathMm(previous, target, 18);
    if (segment) path.push(...(path.length ? segment.slice(1) : segment));
    setModelAngles(previewModel, target);
    const targetPoint = modelEefWorldPosition(previewModel);
    if (targetPoint) {
      const direction = previousPoint
        ? targetPoint.clone().sub(previousPoint).normalize()
        : new THREE.Vector3(0, 0, 1);
      const arrow = new THREE.ArrowHelper(direction, targetPoint, 0.065, 0xffbe55, 0.020, 0.012);
      arrow.name = `handeye-waypoint-${index + 1}`;
      handeyePlanArrows.add(arrow);
      previousPoint = targetPoint;
    }
    previous = target;
  }
  // Independent holdout poses are intentionally not part of the fit. Draw
  // them in blue so the operator can distinguish the final verification pass
  // from the amber training trajectory before confirming execution.
  let validationPrevious = currentEefWorldPosition();
  for (const [index, target] of validationTargets.entries()) {
    setModelAngles(previewModel, target);
    const targetPoint = modelEefWorldPosition(previewModel);
    if (!targetPoint) continue;
    const direction = validationPrevious
      ? targetPoint.clone().sub(validationPrevious).normalize()
      : new THREE.Vector3(0, 0, 1);
    const arrow = new THREE.ArrowHelper(direction, targetPoint, 0.065, 0x5dc7ff, 0.020, 0.012);
    arrow.name = `handeye-validation-waypoint-${index + 1}`;
    handeyePlanArrows.add(arrow);
    validationPrevious = targetPoint;
  }
  handeyePlanArrows.visible = handeyePlanArrows.children.length > 0;
  previewKind = "handeye-plan";
  previewDirty = true;
  previewAngles = targets[0].slice();
  displayedPreviewAngles = previewAngles.slice();
  if (previewModel) setModelAngles(previewModel, previewAngles);
  setPlannedPath(path, "joint");
  updatePreviewVisibility();
  updateInspector();
  const validationLabel = validationTargets.length
    ? `；蓝色 ${validationTargets.length} 个独立验证姿态`
    : "";
  setPreviewStatus(
    `手眼标定预览：琥珀色 ${targets.length} 个采样姿态${validationLabel}；确认后才执行。`,
    "ready",
  );
});
