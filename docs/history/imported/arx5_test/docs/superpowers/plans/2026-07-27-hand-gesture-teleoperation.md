# Hand Gesture Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add low-latency, relative 6DoF hand teleoperation with a two-hand deadman clutch, binary torque-limited gripper control, and server-enforced safety.

**Architecture:** A focused browser module reads the Mac camera and converts two-hand MediaPipe landmarks into relative pose intent. The existing encrypted control WebSocket carries only relative targets to a new server-side `TeleoperationServo`, which owns IK, workspace checks, Ruckig planning, watchdog behavior, and the combined arm/gripper command.

**Tech Stack:** Python 3.12, `unittest`, NumPy, Ruckig, existing ARX5 controller and WebSocket server, browser ES modules, MediaPipe Tasks Vision 0.10.35, Three.js, Node 18 built-in test runner.

---

## Execution Guard

The starting worktree contains unrelated user changes. Do not reset, restore, stash, or
overwrite them. Before each commit, inspect `git diff --cached`; use patch staging for
pre-existing modified files, especially `README.md`, `arx5_control/__init__.py`,
`arx5_control/controller.py`, `docs/SAFETY.md`, `tests/test_cartesian.py`, and
`tests/test_controller_basics.py`. A clean worktree is not a completion requirement;
only planned hunks and new files should enter the feature commits.

## File Map

New files:

- `configs/teleoperation.json`: server-owned workspace, axis mapping, and motion profiles.
- `arx5_control/teleoperation.py`: immutable configuration, workspace checks, profile lookup, and target validation.
- `arx5_control/teleoperation_servo.py`: one 100 Hz worker for arm pose and gripper state.
- `tests/test_teleoperation.py`: configuration and unified-servo tests without hardware.
- `web/hand_teleoperation_math.mjs`: pure landmark geometry, gesture latches, filters, and relative mapping.
- `web/hand_teleoperation.mjs`: Mac camera, MediaPipe loop, browser state machine, overlays, and protocol orchestration.
- `tests/js/hand_teleoperation_math.test.mjs`: Node tests for the pure browser module.
- `tests/test_hand_teleoperation_js.py`: includes the Node suite in `./arx5 test`.

Modified files:

- `arx5_control/controller.py`: pass an optional quaternion through live Cartesian planning.
- `arx5_control/__init__.py`: export teleoperation types.
- `scripts/web_control_server.py`: teleoperation messages, session ownership, profile selection, state, stop, and emergency integration.
- `tests/test_controller_basics.py`: orientation forwarding test.
- `tests/test_web_control.py`: protocol, exclusivity, markup, and lifecycle tests.
- `web/manual_control.js`: teleoperation transport and events on the existing control socket.
- `web/app.js`: pause D455 pose inference while hand teleoperation is active.
- `web/index.html`: hand mode, Mac preview, shared gesture/calibration tool switch, and controls.
- `web/styles.css`: compact desktop/mobile layouts and state styling.
- `web/robot_viewer.js`: hand mode, server-validated Ghost, and TCP safety-box visualization.
- `README.md`, `docs/MEDIAPIPE.md`, `docs/SAFETY.md`, `docs/WEB_CONSOLE.md`: operator workflow and safety boundary.

### Task 1: Hand Geometry Primitives

**Files:**
- Create: `web/hand_teleoperation_math.mjs`
- Create: `tests/js/hand_teleoperation_math.test.mjs`
- Create: `tests/test_hand_teleoperation_js.py`

- [ ] **Step 1: Add the failing Node tests**

Create `tests/js/hand_teleoperation_math.test.mjs`:

```javascript
import test from "node:test";
import assert from "node:assert/strict";
import {
  closestQuaternion,
  isOpenPalm,
  palmFrameQuaternion,
  palmScale,
  pinchRatio,
  relativeOperatorTranslation,
} from "../../web/hand_teleoperation_math.mjs";

const point = (x, y, z = 0) => ({ x, y, z });

function openHand() {
  const hand = Array.from({ length: 21 }, () => point(0, 0, 0));
  hand[0] = point(0, 0, 0);
  hand[5] = point(1, 0, 0);
  hand[9] = point(0.3, 0.2, 0);
  hand[17] = point(-1, 0, 0);
  for (const [mcp, pip, dip] of [[5, 6, 7], [9, 10, 11], [13, 14, 15], [17, 18, 19]]) {
    const x = hand[mcp].x;
    hand[pip] = point(x, 1, 0);
    hand[dip] = point(x, 2, 0);
  }
  hand[4] = point(0, 0, 0);
  hand[8] = point(0.2, 0, 0);
  return hand;
}

test("open palm and normalized pinch are scale independent", () => {
  const hand = openHand();
  assert.equal(isOpenPalm(hand), true);
  assert.equal(palmScale(hand), 1);
  assert.ok(Math.abs(pinchRatio(hand) - 0.1) < 1e-9);
});

test("operator translation uses mirrored right, up, and palm scale", () => {
  const delta = relativeOperatorTranslation(
    { x: 0.40, y: 0.50, scale: 0.20 },
    { x: 0.50, y: 0.45, scale: 0.10 },
  );
  assert.deepEqual(delta.slice(1), [0.04, 0.02]);
  assert.ok(Math.abs(delta[0] - 0.25 * Math.log(2)) < 1e-9);
});

test("palm quaternion is finite and remains on the closest sign", () => {
  const quaternion = palmFrameQuaternion(openHand(), "right");
  assert.equal(quaternion.length, 4);
  assert.ok(quaternion.every(Number.isFinite));
  assert.ok(Math.abs(Math.hypot(...quaternion) - 1) < 1e-9);
  assert.deepEqual(
    closestQuaternion(quaternion.map((value) => -value), quaternion),
    quaternion,
  );
});
```

Create `tests/test_hand_teleoperation_js.py`:

```python
from pathlib import Path
import shutil
import subprocess
import unittest


class HandTeleoperationJavaScriptTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("node"), "Node.js is required")
    def test_node_suite(self) -> None:
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(
            ["node", "--test", "tests/js/hand_teleoperation_math.test.mjs"],
            cwd=root,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and verify the missing module failure**

Run:

```bash
.venv/bin/python -m unittest tests.test_hand_teleoperation_js -v
```

Expected: FAIL because `web/hand_teleoperation_math.mjs` does not exist.

- [ ] **Step 3: Implement finite vector, angle, palm, and quaternion helpers**

Create `web/hand_teleoperation_math.mjs` with these public functions:

```javascript
const EPSILON = 1e-9;

function vector(a, b) {
  return [b.x - a.x, b.y - a.y, b.z - a.z];
}

function dot(a, b) {
  return a.reduce((sum, value, index) => sum + value * b[index], 0);
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function normalize(value) {
  const norm = Math.hypot(...value);
  if (!Number.isFinite(norm) || norm < EPSILON) {
    throw new RangeError("hand basis is degenerate");
  }
  return value.map((component) => component / norm);
}

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function jointAngleDegrees(a, b, c) {
  const first = normalize(vector(b, a));
  const second = normalize(vector(b, c));
  const cosine = Math.max(-1, Math.min(1, dot(first, second)));
  return Math.acos(cosine) * 180 / Math.PI;
}

export function isOpenPalm(hand, minimumDegrees = 155) {
  if (!Array.isArray(hand) || hand.length !== 21) return false;
  return [[5, 6, 7], [9, 10, 11], [13, 14, 15], [17, 18, 19]]
    .every(([mcp, pip, dip]) => (
      jointAngleDegrees(hand[mcp], hand[pip], hand[dip]) >= minimumDegrees
    ));
}

export function palmScale(hand) {
  const values = [[0, 5], [0, 17], [5, 17], [0, 9]]
    .map(([first, second]) => distance(hand[first], hand[second]))
    .sort((a, b) => a - b);
  return 0.5 * (values[1] + values[2]);
}

export function pinchRatio(hand) {
  const width = distance(hand[5], hand[17]);
  return width > EPSILON ? distance(hand[4], hand[8]) / width : Number.NaN;
}

export function relativeOperatorTranslation(origin, current) {
  if (![origin.x, origin.y, origin.scale, current.x, current.y, current.scale]
    .every(Number.isFinite) || origin.scale <= 0 || current.scale <= 0) {
    throw new TypeError("hand translation input is invalid");
  }
  return [
    0.25 * Math.log(origin.scale / current.scale),
    0.40 * (current.x - origin.x),
    0.40 * (origin.y - current.y),
  ];
}

function matrixQuaternion(matrix) {
  const trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  let quaternion;
  if (trace > 0) {
    const scale = 2 * Math.sqrt(trace + 1);
    quaternion = [
      (matrix[2][1] - matrix[1][2]) / scale,
      (matrix[0][2] - matrix[2][0]) / scale,
      (matrix[1][0] - matrix[0][1]) / scale,
      scale / 4,
    ];
  } else {
    const axis = matrix[0][0] > matrix[1][1]
      ? (matrix[0][0] > matrix[2][2] ? 0 : 2)
      : (matrix[1][1] > matrix[2][2] ? 1 : 2);
    const next = (axis + 1) % 3;
    const last = (axis + 2) % 3;
    const scale = 2 * Math.sqrt(1 + matrix[axis][axis] - matrix[next][next] - matrix[last][last]);
    quaternion = [0, 0, 0, 0];
    quaternion[axis] = scale / 4;
    quaternion[next] = (matrix[next][axis] + matrix[axis][next]) / scale;
    quaternion[last] = (matrix[last][axis] + matrix[axis][last]) / scale;
    quaternion[3] = (matrix[last][next] - matrix[next][last]) / scale;
  }
  return normalize(quaternion);
}

export function palmFrameQuaternion(hand, handedness) {
  const across = normalize(vector(hand[17], hand[5]));
  const forwardRaw = normalize(vector(hand[0], hand[9]));
  const forward = normalize(forwardRaw.map(
    (value, index) => value - dot(forwardRaw, across) * across[index],
  ));
  let normal = normalize(cross(across, forward));
  if (String(handedness).toLowerCase() === "left") {
    normal = normal.map((value) => -value);
  }
  const canonicalForward = normalize(cross(normal, across));
  return matrixQuaternion([
    [across[0], canonicalForward[0], normal[0]],
    [across[1], canonicalForward[1], normal[1]],
    [across[2], canonicalForward[2], normal[2]],
  ]);
}

export function closestQuaternion(candidate, reference) {
  return dot(candidate, reference) < 0
    ? candidate.map((value) => -value)
    : candidate.slice();
}
```

- [ ] **Step 4: Run the JavaScript tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_hand_teleoperation_js -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add web/hand_teleoperation_math.mjs tests/js/hand_teleoperation_math.test.mjs tests/test_hand_teleoperation_js.py
git commit -m "feat(web): add hand pose math"
```

### Task 2: Gesture Latches, Filters, and Relative Mapper

**Files:**
- Modify: `web/hand_teleoperation_math.mjs`
- Modify: `tests/js/hand_teleoperation_math.test.mjs`

- [ ] **Step 1: Add failing tests for clutch, pinch, filtering, and mapping**

Append:

```javascript
import {
  ClutchLatch,
  OneEuroVectorFilter,
  PinchLatch,
  TeleoperationMapper,
  transformQuaternionFrame,
} from "../../web/hand_teleoperation_math.mjs";

test("clutch requires five engage frames and two release frames", () => {
  const clutch = new ClutchLatch();
  for (let index = 0; index < 4; index += 1) assert.equal(clutch.update(true), false);
  assert.equal(clutch.update(true), true);
  assert.equal(clutch.update(false), true);
  assert.equal(clutch.update(false), false);
});

test("pinch uses three-frame hysteresis", () => {
  const pinch = new PinchLatch();
  for (let index = 0; index < 2; index += 1) assert.equal(pinch.update(0.2), "open");
  assert.equal(pinch.update(0.2), "close");
  for (let index = 0; index < 2; index += 1) assert.equal(pinch.update(0.6), "close");
  assert.equal(pinch.update(0.6), "open");
});

test("mapper starts at zero and applies the signed axis matrix", () => {
  const mapper = new TeleoperationMapper({
    operatorToBase: [[1, 0, 0], [0, -1, 0], [0, 0, 1]],
    translationGain: 1,
    rotationGain: 1,
  });
  mapper.engage({ wrist: { x: 0.5, y: 0.5 }, scale: 0.2, quaternion: [0, 0, 0, 1] }, 0);
  const target = mapper.update(
    { wrist: { x: 0.6, y: 0.45 }, scale: 0.2, quaternion: [0, 0, 0, 1] },
    1 / 30,
  );
  assert.ok(Math.abs(target.delta_xyz_m[0]) < 1e-9);
  assert.ok(target.delta_xyz_m[1] < 0);
  assert.ok(target.delta_xyz_m[2] > 0);
  assert.deepEqual(target.delta_orientation_xyzw, [0, 0, 0, 1]);
});

test("One Euro filter rejects non-monotonic timestamps", () => {
  const filter = new OneEuroVectorFilter();
  filter.update([0, 0, 0], 1);
  assert.throws(() => filter.update([1, 1, 1], 1), /timestamp/);
});

test("rotation is conjugated into the signed base frame", () => {
  const transformed = transformQuaternionFrame(
    [0, 0, Math.SQRT1_2, Math.SQRT1_2],
    [[1, 0, 0], [0, -1, 0], [0, 0, 1]],
  );
  assert.ok(transformed[2] < 0);
  assert.ok(transformed[3] > 0);
});

test("mapper caps each browser frame in translation and rotation", () => {
  const mapper = new TeleoperationMapper({
    operatorToBase: [[1, 0, 0], [0, -1, 0], [0, 0, 1]],
    translationGain: 1,
    rotationGain: 1,
    maxTranslationStepM: 0.005,
    maxRotationStepRad: 3 * Math.PI / 180,
  });
  mapper.engage(
    { wrist: { x: 0.5, y: 0.5 }, scale: 0.2, quaternion: [0, 0, 0, 1] },
    0,
  );
  const target = mapper.update({
    wrist: { x: 0.9, y: 0.1 },
    scale: 0.1,
    quaternion: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
  }, 1 / 30);
  assert.ok(Math.hypot(...target.delta_xyz_m) <= 0.005 + 1e-12);
  const rotation = 2 * Math.acos(Math.min(1, Math.abs(
    target.delta_orientation_xyzw[3],
  )));
  assert.ok(rotation <= 3 * Math.PI / 180 + 1e-12);
});
```

- [ ] **Step 2: Run the suite and verify missing exports**

Run:

```bash
node --test tests/js/hand_teleoperation_math.test.mjs
```

Expected: FAIL because the four classes are not exported.

- [ ] **Step 3: Implement the state latches**

Append to `web/hand_teleoperation_math.mjs`:

```javascript
export class ClutchLatch {
  constructor(engageFrames = 5, releaseFrames = 2) {
    this.engageFrames = engageFrames;
    this.releaseFrames = releaseFrames;
    this.reset();
  }

  reset() {
    this.engaged = false;
    this.positive = 0;
    this.negative = 0;
  }

  update(open) {
    this.positive = open ? this.positive + 1 : 0;
    this.negative = open ? 0 : this.negative + 1;
    if (!this.engaged && this.positive >= this.engageFrames) this.engaged = true;
    if (this.engaged && this.negative >= this.releaseFrames) this.engaged = false;
    return this.engaged;
  }
}

export class PinchLatch {
  constructor(closeThreshold = 0.32, openThreshold = 0.48, frames = 3) {
    this.closeThreshold = closeThreshold;
    this.openThreshold = openThreshold;
    this.frames = frames;
    this.state = "open";
    this.count = 0;
  }

  reset(state = "open") {
    this.state = state;
    this.count = 0;
  }

  update(ratio) {
    const requested = ratio < this.closeThreshold
      ? "close"
      : ratio > this.openThreshold ? "open" : this.state;
    this.count = requested === this.state ? 0 : this.count + 1;
    if (requested !== this.state && this.count >= this.frames) {
      this.state = requested;
      this.count = 0;
    }
    return this.state;
  }
}
```

- [ ] **Step 4: Implement quaternion composition, One Euro filtering, and mapper**

Add these exported helpers, then add the filters and mapper:

```javascript
function normalizedQuaternion(value) {
  if (!Array.isArray(value) || value.length !== 4 || !value.every(Number.isFinite)) {
    throw new TypeError("quaternion must contain four finite values");
  }
  return normalize(value);
}

export function quaternionMultiply(first, second) {
  const [ax, ay, az, aw] = normalizedQuaternion(first);
  const [bx, by, bz, bw] = normalizedQuaternion(second);
  return normalizedQuaternion([
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]);
}

export function quaternionInverse(value) {
  const [x, y, z, w] = normalizedQuaternion(value);
  return [-x, -y, -z, w];
}

export function quaternionSlerp(first, second, fraction) {
  const start = normalizedQuaternion(first);
  let target = normalizedQuaternion(second);
  let cosine = dot(start, target);
  if (cosine < 0) {
    target = target.map((value) => -value);
    cosine = -cosine;
  }
  cosine = Math.max(-1, Math.min(1, cosine));
  if (cosine > 0.9995) {
    return normalizedQuaternion(start.map(
      (value, index) => value + fraction * (target[index] - value),
    ));
  }
  const angle = Math.acos(cosine);
  const sine = Math.sin(angle);
  return normalizedQuaternion(start.map((value, index) => (
    Math.sin((1 - fraction) * angle) / sine * value
    + Math.sin(fraction * angle) / sine * target[index]
  )));
}

export function quaternionDistanceRadians(first, second) {
  const start = normalizedQuaternion(first);
  const target = normalizedQuaternion(second);
  return 2 * Math.acos(Math.max(-1, Math.min(1, Math.abs(dot(start, target)))));
}

function clampVectorStep(previous, desired, maximumStep) {
  const step = desired.map((value, index) => value - previous[index]);
  const distance = Math.hypot(...step);
  const scale = distance > maximumStep ? maximumStep / distance : 1;
  return previous.map((value, index) => value + scale * step[index]);
}

function quaternionMatrix(value) {
  const [x, y, z, w] = normalizedQuaternion(value);
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

function multiplyMatrices(first, second) {
  return first.map((row) => second[0].map((_, column) => (
    row.reduce((sum, value, index) => sum + value * second[index][column], 0)
  )));
}

export function transformQuaternionFrame(value, frame) {
  if (!Array.isArray(frame) || frame.length !== 3
      || !frame.every((row) => Array.isArray(row) && row.length === 3)) {
    throw new TypeError("quaternion frame must be a 3x3 matrix");
  }
  const transpose = frame[0].map((_, column) => frame.map((row) => row[column]));
  return matrixQuaternion(multiplyMatrices(
    multiplyMatrices(frame, quaternionMatrix(value)),
    transpose,
  ));
}
```

```javascript
class OneEuroScalarFilter {
  constructor(minCutoff = 1.5, beta = 0.20, derivativeCutoff = 1.0) {
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.derivativeCutoff = derivativeCutoff;
    this.value = null;
    this.derivative = 0;
    this.timestamp = null;
  }

  alpha(cutoff, deltaSeconds) {
    const tau = 1 / (2 * Math.PI * cutoff);
    return 1 / (1 + tau / deltaSeconds);
  }

  update(value, timestamp) {
    if (this.timestamp === null) {
      this.value = value;
      this.timestamp = timestamp;
      return value;
    }
    const deltaSeconds = timestamp - this.timestamp;
    if (!(deltaSeconds > 0)) throw new RangeError("filter timestamp must increase");
    const rawDerivative = (value - this.value) / deltaSeconds;
    const derivativeAlpha = this.alpha(this.derivativeCutoff, deltaSeconds);
    this.derivative += derivativeAlpha * (rawDerivative - this.derivative);
    const cutoff = this.minCutoff + this.beta * Math.abs(this.derivative);
    const valueAlpha = this.alpha(cutoff, deltaSeconds);
    this.value += valueAlpha * (value - this.value);
    this.timestamp = timestamp;
    return this.value;
  }
}

export class OneEuroVectorFilter {
  constructor() {
    this.filters = [0, 1, 2].map(() => new OneEuroScalarFilter());
  }

  update(value, timestamp) {
    return this.filters.map((filter, index) => filter.update(value[index], timestamp));
  }
}

export class TeleoperationMapper {
  constructor({
    operatorToBase,
    translationGain,
    rotationGain,
    maxTranslationStepM = 0.005,
    maxRotationStepRad = 3 * Math.PI / 180,
    rotationTimeConstantS = 0.05,
  }) {
    this.operatorToBase = operatorToBase;
    this.translationGain = translationGain;
    this.rotationGain = rotationGain;
    this.maxTranslationStepM = maxTranslationStepM;
    this.maxRotationStepRad = maxRotationStepRad;
    this.rotationTimeConstantS = rotationTimeConstantS;
    this.translationFilter = new OneEuroVectorFilter();
    this.origin = null;
    this.previousTranslation = [0, 0, 0];
    this.previousQuaternion = [0, 0, 0, 1];
    this.previousTimestamp = null;
  }

  engage(handPose, timestamp) {
    this.origin = structuredClone(handPose);
    this.translationFilter = new OneEuroVectorFilter();
    this.translationFilter.update([0, 0, 0], timestamp);
    this.previousTranslation = [0, 0, 0];
    this.previousQuaternion = [0, 0, 0, 1];
    this.previousTimestamp = timestamp;
  }

  update(handPose, timestamp) {
    if (!this.origin) throw new Error("teleoperation mapper is not engaged");
    const operator = relativeOperatorTranslation(
      { x: this.origin.wrist.x, y: this.origin.wrist.y, scale: this.origin.scale },
      { x: handPose.wrist.x, y: handPose.wrist.y, scale: handPose.scale },
    );
    const base = this.operatorToBase.map(
      (row) => this.translationGain * row.reduce(
        (sum, value, index) => sum + value * operator[index],
        0,
      ),
    );
    const filtered = this.translationFilter.update(base, timestamp);
    const deadzoned = filtered.map((value) => Math.abs(value) < 0.002 ? 0 : value);
    this.previousTranslation = clampVectorStep(
      this.previousTranslation,
      deadzoned,
      this.maxTranslationStepM,
    );
    const operatorRotation = quaternionMultiply(
      handPose.quaternion,
      quaternionInverse(this.origin.quaternion),
    );
    const rawRotation = transformQuaternionFrame(
      operatorRotation,
      this.operatorToBase,
    );
    const scaledRotation = quaternionSlerp(
      [0, 0, 0, 1],
      rawRotation,
      this.rotationGain,
    );
    const desiredRotation = quaternionDistanceRadians(
      [0, 0, 0, 1],
      scaledRotation,
    ) < 1.5 * Math.PI / 180 ? [0, 0, 0, 1] : scaledRotation;
    const elapsed = timestamp - this.previousTimestamp;
    const filteredRotation = quaternionSlerp(
      this.previousQuaternion,
      closestQuaternion(desiredRotation, this.previousQuaternion),
      1 - Math.exp(-elapsed / this.rotationTimeConstantS),
    );
    const rotationStep = quaternionDistanceRadians(
      this.previousQuaternion,
      filteredRotation,
    );
    this.previousQuaternion = closestQuaternion(quaternionSlerp(
      this.previousQuaternion,
      filteredRotation,
      rotationStep > this.maxRotationStepRad
        ? this.maxRotationStepRad / rotationStep : 1,
    ), this.previousQuaternion);
    this.previousTimestamp = timestamp;
    return {
      delta_xyz_m: this.previousTranslation.slice(),
      delta_orientation_xyzw: this.previousQuaternion.slice(),
    };
  }
}
```

The quaternion helpers must validate four finite values, normalize their outputs,
use the shortest SLERP arc, and return XYZW order.

- [ ] **Step 5: Run both test entry points**

Run:

```bash
node --test tests/js/hand_teleoperation_math.test.mjs
.venv/bin/python -m unittest tests.test_hand_teleoperation_js -v
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add web/hand_teleoperation_math.mjs tests/js/hand_teleoperation_math.test.mjs
git commit -m "feat(web): map filtered hand motion"
```

### Task 3: Server-Owned Teleoperation Configuration

**Files:**
- Create: `configs/teleoperation.json`
- Create: `arx5_control/teleoperation.py`
- Create: `tests/test_teleoperation.py`
- Modify: `arx5_control/__init__.py`

- [ ] **Step 1: Write failing configuration and workspace tests**

Create `tests/test_teleoperation.py` with:

```python
from pathlib import Path
import tempfile
import unittest

import numpy as np

from arx5_control.teleoperation import load_teleoperation_config


class TeleoperationConfigTests(unittest.TestCase):
    def test_project_defaults_load_and_profiles_match_design(self) -> None:
        config = load_teleoperation_config(Path("configs/teleoperation.json"))
        self.assertEqual(set(config.profiles), {"fine", "fast"})
        self.assertAlmostEqual(config.profiles["fine"].joint_velocity_rad_s, 0.15)
        self.assertAlmostEqual(config.profiles["fast"].tcp_linear_velocity_m_s, 0.10)
        np.testing.assert_allclose(
            config.operator_to_base,
            [[1, 0, 0], [0, -1, 0], [0, 0, 1]],
        )

    def test_workspace_rejects_outside_and_scales_inside_soft_boundary(self) -> None:
        workspace = load_teleoperation_config(
            Path("configs/teleoperation.json")
        ).workspace
        self.assertTrue(workspace.contains(np.array([0.20, 0.0, 0.20])))
        self.assertFalse(workspace.contains(np.array([0.14, 0.0, 0.20])))
        self.assertAlmostEqual(
            workspace.speed_scale(np.array([0.165, 0.0, 0.20])),
            0.5,
        )
        self.assertAlmostEqual(
            workspace.speed_scale(np.array([0.30, 0.0, 0.20])),
            1.0,
        )

    def test_invalid_axis_matrix_is_rejected(self) -> None:
        invalid = """{
          "operator_to_base": [[1, 0, 0], [1, 0, 0], [0, 0, 1]],
          "workspace": {
            "minimum_m": [0.15, -0.35, 0.08],
            "maximum_m": [0.65, 0.35, 0.55],
            "soft_boundary_m": 0.03
          },
          "profiles": {
            "fine": {
              "translation_gain": 0.5,
              "rotation_gain": 0.6,
              "tcp_linear_velocity_m_s": 0.03,
              "tcp_angular_velocity_rad_s": 0.35,
              "joint_velocity_rad_s": 0.15,
              "joint_acceleration_rad_s2": 0.30,
              "joint_jerk_rad_s3": 1.0
            }
          }
        }"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleoperation.json"
            path.write_text(invalid, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "signed permutation"):
                load_teleoperation_config(path)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify import failure**

Run:

```bash
.venv/bin/python -m unittest tests.test_teleoperation.TeleoperationConfigTests -v
```

Expected: FAIL because `arx5_control.teleoperation` is missing.

- [ ] **Step 3: Add the exact JSON defaults**

Create `configs/teleoperation.json`:

```json
{
  "operator_to_base": [[1, 0, 0], [0, -1, 0], [0, 0, 1]],
  "workspace": {
    "minimum_m": [0.15, -0.35, 0.08],
    "maximum_m": [0.65, 0.35, 0.55],
    "soft_boundary_m": 0.03
  },
  "profiles": {
    "fine": {
      "translation_gain": 0.5,
      "rotation_gain": 0.6,
      "tcp_linear_velocity_m_s": 0.03,
      "tcp_angular_velocity_rad_s": 0.35,
      "joint_velocity_rad_s": 0.15,
      "joint_acceleration_rad_s2": 0.30,
      "joint_jerk_rad_s3": 1.0
    },
    "fast": {
      "translation_gain": 1.0,
      "rotation_gain": 1.0,
      "tcp_linear_velocity_m_s": 0.10,
      "tcp_angular_velocity_rad_s": 0.80,
      "joint_velocity_rad_s": 0.35,
      "joint_acceleration_rad_s2": 0.70,
      "joint_jerk_rad_s3": 3.0
    }
  }
}
```

- [ ] **Step 4: Implement dataclasses and strict loader**

Create `arx5_control/teleoperation.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def _finite_vector(value: Any, length: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {length} finite values")
    return vector.copy()


@dataclass(frozen=True)
class TeleoperationProfile:
    translation_gain: float
    rotation_gain: float
    tcp_linear_velocity_m_s: float
    tcp_angular_velocity_rad_s: float
    joint_velocity_rad_s: float
    joint_acceleration_rad_s2: float
    joint_jerk_rad_s3: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) and value > 0 for value in self.__dict__.values()):
            raise ValueError("teleoperation profile values must be positive and finite")
        if self.translation_gain > 1 or self.rotation_gain > 1:
            raise ValueError("teleoperation gains must not exceed 1")


@dataclass(frozen=True)
class WorkspaceBounds:
    minimum_m: np.ndarray
    maximum_m: np.ndarray
    soft_boundary_m: float

    def __post_init__(self) -> None:
        minimum = _finite_vector(self.minimum_m, 3, "workspace minimum")
        maximum = _finite_vector(self.maximum_m, 3, "workspace maximum")
        if np.any(minimum >= maximum):
            raise ValueError("workspace minimum must be below maximum")
        if not math.isfinite(self.soft_boundary_m) or self.soft_boundary_m <= 0:
            raise ValueError("soft boundary must be positive and finite")
        if np.any(2 * self.soft_boundary_m >= maximum - minimum):
            raise ValueError("soft boundary must be less than half each workspace axis")
        object.__setattr__(self, "minimum_m", minimum)
        object.__setattr__(self, "maximum_m", maximum)

    def contains(self, position_m: np.ndarray) -> bool:
        point = _finite_vector(position_m, 3, "TCP position")
        return bool(np.all(point >= self.minimum_m) and np.all(point <= self.maximum_m))

    def speed_scale(self, position_m: np.ndarray) -> float:
        point = _finite_vector(position_m, 3, "TCP position")
        if not self.contains(point):
            return 0.0
        clearance = np.minimum(point - self.minimum_m, self.maximum_m - point)
        return float(np.clip(np.min(clearance) / self.soft_boundary_m, 0.0, 1.0))


@dataclass(frozen=True)
class TeleoperationConfig:
    operator_to_base: np.ndarray
    workspace: WorkspaceBounds
    profiles: dict[str, TeleoperationProfile]

    def __post_init__(self) -> None:
        matrix = np.asarray(self.operator_to_base, dtype=np.float64).copy()
        valid_entries = np.all(np.isin(matrix, (-1.0, 0.0, 1.0)))
        valid_axes = (
            matrix.shape == (3, 3)
            and valid_entries
            and np.all(np.sum(np.abs(matrix), axis=0) == 1)
            and np.all(np.sum(np.abs(matrix), axis=1) == 1)
        )
        if not valid_axes:
            raise ValueError("operator_to_base must be a signed permutation matrix")
        if set(self.profiles) != {"fine", "fast"}:
            raise ValueError("teleoperation profiles must contain fine and fast")
        object.__setattr__(self, "operator_to_base", matrix)

    def web_payload(self) -> dict[str, Any]:
        return {
            "operator_to_base": self.operator_to_base.tolist(),
            "workspace": {
                "minimum_m": self.workspace.minimum_m.tolist(),
                "maximum_m": self.workspace.maximum_m.tolist(),
                "soft_boundary_m": self.workspace.soft_boundary_m,
            },
            "profiles": {
                name: profile.__dict__.copy()
                for name, profile in self.profiles.items()
            },
        }


def load_teleoperation_config(path: Path) -> TeleoperationConfig:
    raw = json.loads(path.read_text(encoding="utf-8"))
    workspace = WorkspaceBounds(
        minimum_m=raw["workspace"]["minimum_m"],
        maximum_m=raw["workspace"]["maximum_m"],
        soft_boundary_m=float(raw["workspace"]["soft_boundary_m"]),
    )
    profiles = {
        name: TeleoperationProfile(**values)
        for name, values in raw["profiles"].items()
    }
    return TeleoperationConfig(
        operator_to_base=raw["operator_to_base"],
        workspace=workspace,
        profiles=profiles,
    )
```

Export `TeleoperationConfig`, `TeleoperationProfile`, `WorkspaceBounds`, and
`load_teleoperation_config` from `arx5_control/__init__.py`.

- [ ] **Step 5: Run the configuration tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_teleoperation.TeleoperationConfigTests -v
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add configs/teleoperation.json arx5_control/teleoperation.py tests/test_teleoperation.py
git add -p arx5_control/__init__.py
git diff --cached --check
git commit -m "feat(control): add teleoperation safety config"
```

### Task 4: Orientation-Aware Live Cartesian Planning

**Files:**
- Modify: `arx5_control/controller.py`
- Modify: `tests/test_controller_basics.py`
- Modify: `tests/test_servo.py`

- [ ] **Step 1: Write a failing orientation-forwarding test**

Add to `tests/test_controller_basics.py`:

```python
def test_cartesian_servo_forwards_target_orientation(self) -> None:
    controller = ArmController()
    controller._mode = ControllerMode.DAMPING  # noqa: SLF001
    expected = object()

    class Planner:
        def plan_relative_xyz(self, start_pose, start_joint, delta, **kwargs):
            np.testing.assert_allclose(
                kwargs["target_orientation_xyzw"],
                [0.0, 0.0, 0.1, 0.9949874371],
            )
            np.testing.assert_allclose(kwargs["seed_joint_position"], np.zeros(6))
            self.unrestricted = kwargs["unrestricted"]
            return expected

    planner = Planner()
    controller._cartesian_planner = planner  # noqa: SLF001
    result = controller.plan_cartesian_servo_target(
        np.zeros(6),
        np.zeros(6),
        np.array([0.01, 0.0, 0.0]),
        np.zeros(6),
        target_orientation_xyzw=np.array([0.0, 0.0, 0.1, 0.9949874371]),
    )
    self.assertIs(result, expected)
    self.assertTrue(planner.unrestricted)
```

- [ ] **Step 2: Run the test and verify signature failure**

Run:

```bash
.venv/bin/python -m unittest tests.test_controller_basics.ControllerBasicsTests.test_cartesian_servo_forwards_target_orientation -v
```

Expected: FAIL with an unexpected `target_orientation_xyzw` argument.

- [ ] **Step 3: Extend the controller method without changing existing callers**

Change `ArmController.plan_cartesian_servo_target` to:

```python
def plan_cartesian_servo_target(
    self,
    start_pose_6d: np.ndarray,
    start_joint_position: np.ndarray,
    delta_xyz_m: np.ndarray,
    seed_joint_position: np.ndarray,
    *,
    target_orientation_xyzw: np.ndarray | None = None,
) -> CartesianPlan:
    try:
        return self._require_cartesian_planner().plan_relative_xyz(
            np.asarray(start_pose_6d, dtype=np.float64),
            np.asarray(start_joint_position, dtype=np.float64),
            np.asarray(delta_xyz_m, dtype=np.float64),
            target_orientation_xyzw=target_orientation_xyzw,
            seed_joint_position=np.asarray(seed_joint_position, dtype=np.float64),
            unrestricted=True,
        )
    except CartesianPlanningError as exc:
        raise SafetyError(str(exc)) from exc
```

This method is planning-only and sends no SDK command, so it intentionally accepts both
ACTIVE and DAMPING callers. `CartesianServo` and `TeleoperationServo` retain their own
ACTIVE-mode guards before any online command.

Update every fake `plan_cartesian_servo_target` in `tests/test_servo.py` to accept the
same keyword-only argument and ignore it for existing XYZ-only tests.

- [ ] **Step 4: Run controller, Cartesian, and existing servo tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_controller_basics tests.test_cartesian tests.test_servo -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_servo.py
git add -p arx5_control/controller.py tests/test_controller_basics.py
git diff --cached --check
git commit -m "feat(control): support 6DoF servo targets"
```

### Task 5: Unified Arm and Gripper Teleoperation Servo

**Files:**
- Create: `arx5_control/teleoperation_servo.py`
- Modify: `arx5_control/teleoperation.py`
- Modify: `arx5_control/__init__.py`
- Modify: `tests/test_teleoperation.py`

- [ ] **Step 1: Add failing target-validation tests**

Add to `tests/test_teleoperation.py`:

```python
import math

from arx5_control.teleoperation import (
    GripperState,
    TeleoperationTarget,
    limit_teleoperation_target_step,
    quaternion_distance_rad,
)


class TeleoperationTargetTests(unittest.TestCase):
    def test_target_normalizes_quaternion_and_copies_delta(self) -> None:
        source_delta = np.array([0.01, 0.02, 0.03])
        target = TeleoperationTarget(
            delta_xyz_m=source_delta,
            delta_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.00001]),
            gripper_state=GripperState.OPEN,
        )
        source_delta[0] = 99.0
        np.testing.assert_allclose(target.delta_xyz_m, [0.01, 0.02, 0.03])
        np.testing.assert_allclose(np.linalg.norm(target.delta_orientation_xyzw), 1.0)
        self.assertEqual(target.gripper_state, GripperState.OPEN)

    def test_target_rejects_non_finite_values(self) -> None:
        with self.assertRaisesRegex(ValueError, "delta"):
            TeleoperationTarget(
                delta_xyz_m=np.array([np.nan, 0.0, 0.0]),
                delta_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                gripper_state=GripperState.CLOSE,
            )
        with self.assertRaisesRegex(ValueError, "norm"):
            TeleoperationTarget(
                delta_xyz_m=np.zeros(3),
                delta_orientation_xyzw=np.array([0.0, 0.0, 0.0, 2.0]),
                gripper_state=GripperState.OPEN,
            )

    def test_pose_slew_uses_server_linear_and_angular_caps(self) -> None:
        config = load_teleoperation_config(Path("configs/teleoperation.json"))
        previous = TeleoperationTarget(
            np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), GripperState.OPEN
        )
        desired = TeleoperationTarget(
            np.array([0.30, 0.0, 0.0]),
            np.array([0.0, 0.0, math.sin(0.5), math.cos(0.5)]),
            GripperState.CLOSE,
        )
        limited = limit_teleoperation_target_step(
            previous, desired, config.profiles["fine"], 0.10
        )
        self.assertAlmostEqual(np.linalg.norm(limited.delta_xyz_m), 0.003)
        self.assertAlmostEqual(
            quaternion_distance_rad(
                previous.delta_orientation_xyzw,
                limited.delta_orientation_xyzw,
            ),
            0.035,
        )
        self.assertEqual(limited.gripper_state, GripperState.CLOSE)
```

- [ ] **Step 2: Run the target tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_teleoperation.TeleoperationTargetTests -v
```

Expected: FAIL because `GripperState` and `TeleoperationTarget` are missing.

- [ ] **Step 3: Add immutable target types**

Append to `arx5_control/teleoperation.py`:

```python
from enum import Enum

class GripperState(str, Enum):
    OPEN = "open"
    CLOSE = "close"


@dataclass(frozen=True)
class TeleoperationTarget:
    delta_xyz_m: np.ndarray
    delta_orientation_xyzw: np.ndarray
    gripper_state: GripperState

    def __post_init__(self) -> None:
        delta = _finite_vector(self.delta_xyz_m, 3, "teleoperation delta")
        orientation = _finite_vector(
            self.delta_orientation_xyzw, 4, "teleoperation quaternion"
        )
        norm = float(np.linalg.norm(orientation))
        if not 0.95 <= norm <= 1.05:
            raise ValueError("teleoperation quaternion norm must be near one")
        orientation = orientation / norm
        object.__setattr__(self, "delta_xyz_m", delta)
        object.__setattr__(self, "delta_orientation_xyzw", orientation)
        object.__setattr__(self, "gripper_state", GripperState(self.gripper_state))


def quaternion_multiply_xyzw(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = _finite_vector(first, 4, "first quaternion")
    bx, by, bz, bw = _finite_vector(second, 4, "second quaternion")
    result = np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])
    return result / np.linalg.norm(result)


def quaternion_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    a = _finite_vector(first, 4, "first quaternion")
    b = _finite_vector(second, 4, "second quaternion")
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    return 2.0 * math.acos(float(np.clip(abs(np.dot(a, b)), -1.0, 1.0)))


def quaternion_slerp_xyzw(
    first: np.ndarray, second: np.ndarray, fraction: float
) -> np.ndarray:
    a = _finite_vector(first, 4, "first quaternion")
    b = _finite_vector(second, 4, "second quaternion")
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    dot = float(np.dot(a, b))
    if dot < 0:
        b = -b
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = a + fraction * (b - a)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    result = (
        math.sin((1.0 - fraction) * angle) / math.sin(angle) * a
        + math.sin(fraction * angle) / math.sin(angle) * b
    )
    return result / np.linalg.norm(result)


def limit_teleoperation_target_step(
    previous: TeleoperationTarget,
    desired: TeleoperationTarget,
    profile: TeleoperationProfile,
    elapsed_s: float,
) -> TeleoperationTarget:
    elapsed = float(np.clip(elapsed_s, 1.0 / 60.0, 0.20))
    translation_step = desired.delta_xyz_m - previous.delta_xyz_m
    distance = float(np.linalg.norm(translation_step))
    maximum_distance = profile.tcp_linear_velocity_m_s * elapsed
    if distance > maximum_distance:
        translation_step *= maximum_distance / distance
    angle = quaternion_distance_rad(
        previous.delta_orientation_xyzw,
        desired.delta_orientation_xyzw,
    )
    maximum_angle = profile.tcp_angular_velocity_rad_s * elapsed
    fraction = 1.0 if angle <= maximum_angle or angle < 1e-9 else maximum_angle / angle
    return TeleoperationTarget(
        previous.delta_xyz_m + translation_step,
        quaternion_slerp_xyzw(
            previous.delta_orientation_xyzw,
            desired.delta_orientation_xyzw,
            fraction,
        ),
        desired.gripper_state,
    )
```

- [ ] **Step 4: Add failing unified-worker tests**

Add these imports and fake before the worker tests:

```python
from types import SimpleNamespace
import time

from arx5_control import CartesianPlan, ControllerMode, TeleoperationServo
from arx5_control.cartesian import quaternion_xyzw_to_rpy


class FakeTeleoperationArm:
    def __init__(self) -> None:
        self.mode = ControllerMode.ACTIVE
        self.position = np.zeros(6)
        self.velocity = np.zeros(6)
        self.gripper_position = 0.02
        self.gripper_torque = 0.0
        self.commands = []
        self.emergency_reason = None
        self.last_target_orientation = None
        self.fixed_timestamp = None

    def get_state(self):
        return SimpleNamespace(
            timestamp=(
                time.monotonic()
                if self.fixed_timestamp is None else self.fixed_timestamp
            ),
            joint_position=self.position.copy(),
            joint_velocity=self.velocity.copy(),
            joint_torque=np.zeros(6),
            gripper_position=self.gripper_position,
            gripper_velocity=0.0,
            gripper_torque=self.gripper_torque,
            eef_pose_6d=np.array([0.30, 0.0, 0.20, 0.0, 0.0, 0.0]),
            joint_acceleration=np.zeros(6),
        )

    def get_limits(self):
        return SimpleNamespace(
            joint_position_min=np.full(6, -1.0),
            joint_position_max=np.full(6, 1.0),
            joint_velocity_max=np.full(6, 1.0),
            gripper_width_m=0.088,
        )

    def plan_cartesian_servo_target(
        self,
        start_pose,
        start_joint,
        delta,
        seed,
        *,
        target_orientation_xyzw=None,
    ):
        self.last_target_orientation = np.asarray(target_orientation_xyzw).copy()
        pose = np.asarray(start_pose).copy()
        pose[:3] += np.asarray(delta)
        pose[3:] = quaternion_xyzw_to_rpy(self.last_target_orientation)
        target_joint = np.asarray(start_joint).copy()
        target_joint[:3] += np.asarray(delta)
        return CartesianPlan(
            start_pose_6d=np.asarray(start_pose).copy(),
            target_pose_6d=pose,
            solved_pose_6d=pose.copy(),
            start_joint_position=np.asarray(start_joint).copy(),
            target_joint_position=target_joint,
            position_error_m=0.0,
            orientation_error_rad=0.0,
        )

    def command_joint_servo(self, position, velocity, gripper_position):
        self.position = np.asarray(position).copy()
        self.velocity = np.asarray(velocity).copy()
        self.gripper_position = float(gripper_position)
        self.commands.append(SimpleNamespace(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            gripper_position=self.gripper_position,
        ))
        return self.get_state()

    def emergency_damping(self, reason):
        self.emergency_reason = reason
        self.mode = ControllerMode.FAULT
```

Add tests:

```python
class TeleoperationServoTests(unittest.TestCase):
    def make_servo(self, arm, watchdog=0.20):
        config = load_teleoperation_config(Path("configs/teleoperation.json"))
        return TeleoperationServo(
            arm,
            config=config,
            profile_name="fine",
            rate_hz=100.0,
            watchdog_timeout_s=watchdog,
            state_stale_timeout_s=0.10,
            gripper_velocity_m_s=0.06,
            gripper_acceleration_m_s2=0.60,
            gripper_jerk_m_s3=6.0,
        )

    def test_tracks_pose_and_opens_gripper_in_one_command_loop(self) -> None:
        arm = FakeTeleoperationArm()
        servo = self.make_servo(arm)
        servo.start(open_width_m=0.04, closing_torque_nm=0.15)
        generation = servo.update_target(TeleoperationTarget(
            delta_xyz_m=np.array([0.01, 0.0, 0.0]),
            delta_orientation_xyzw=np.array([0.0, 0.0, 0.1, 0.9949874371]),
            gripper_state=GripperState.OPEN,
        ))
        status = servo.wait_for_target(generation, 0.25)
        servo.stop(timeout_s=2.0)
        self.assertEqual(status.target_generation, generation)
        self.assertTrue(any(command.gripper_position > 0.02 for command in arm.commands))
        self.assertIsNotNone(arm.last_target_orientation)

    def test_target_outside_workspace_is_rejected_without_fault(self) -> None:
        arm = FakeTeleoperationArm()
        servo = self.make_servo(arm)
        servo.start(open_width_m=0.04, closing_torque_nm=0.15)
        generation = servo.update_target(TeleoperationTarget(
            delta_xyz_m=np.array([1.0, 0.0, 0.0]),
            delta_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            gripper_state=GripperState.OPEN,
        ))
        status = servo.wait_for_target(generation, 0.25)
        servo.stop(timeout_s=2.0)
        self.assertIn("workspace", status.target_error)
        self.assertIsNone(arm.emergency_reason)

    def test_close_latches_at_torque_limit_until_open_command(self) -> None:
        arm = FakeTeleoperationArm()
        arm.gripper_torque = 0.15
        servo = self.make_servo(arm)
        servo.start(open_width_m=0.04, closing_torque_nm=0.15)
        generation = servo.update_target(TeleoperationTarget(
            delta_xyz_m=np.zeros(3),
            delta_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            gripper_state=GripperState.CLOSE,
        ))
        status = servo.wait_for_target(generation, 0.25)
        self.assertTrue(status.gripper_contact_latched)
        contact_command_count = len(arm.commands)

        arm.gripper_torque = 0.0
        generation = servo.update_target(TeleoperationTarget(
            delta_xyz_m=np.zeros(3),
            delta_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            gripper_state=GripperState.OPEN,
        ))
        status = servo.wait_for_target(generation, 0.25)
        servo.stop(timeout_s=2.0)
        self.assertFalse(status.gripper_contact_latched)
        self.assertTrue(any(
            command.gripper_position > 0.02
            for command in arm.commands[contact_command_count:]
        ))

    def test_watchdog_brakes_both_planners(self) -> None:
        arm = FakeTeleoperationArm()
        servo = self.make_servo(arm, watchdog=0.05)
        servo.start(open_width_m=0.04, closing_torque_nm=0.15)
        deadline = time.monotonic() + 1.0
        while servo.status().active and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(servo.status().active)
        self.assertEqual(servo.status().stop_reason, "target watchdog timeout")
        self.assertIsNone(arm.emergency_reason)

    def test_stale_feedback_enters_fault_damping(self) -> None:
        arm = FakeTeleoperationArm()
        arm.fixed_timestamp = 1.0
        servo = self.make_servo(arm, watchdog=0.50)
        servo.start(open_width_m=0.04, closing_torque_nm=0.15)
        deadline = time.monotonic() + 1.0
        while arm.emergency_reason is None and time.monotonic() < deadline:
            time.sleep(0.01)
        servo.stop(timeout_s=2.0)
        self.assertEqual(arm.mode, ControllerMode.FAULT)
        self.assertIn("timestamp", arm.emergency_reason)
```

- [ ] **Step 5: Run the worker tests and verify import failure**

Run:

```bash
.venv/bin/python -m unittest tests.test_teleoperation.TeleoperationServoTests -v
```

Expected: FAIL because `TeleoperationServo` is missing.

- [ ] **Step 6: Implement the public worker interface and status**

Create `arx5_control/teleoperation_servo.py` with:

```python
class TeleoperationArm(Protocol):
    mode: ControllerMode

    def get_state(self) -> ArmSnapshot: ...
    def get_limits(self): ...
    def plan_cartesian_servo_target(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        delta_xyz_m: np.ndarray,
        seed_joint_position: np.ndarray,
        *,
        target_orientation_xyzw: np.ndarray | None = None,
    ): ...
    def command_joint_servo(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        gripper_position: float,
    ) -> ArmSnapshot: ...
    def emergency_damping(self, reason: str) -> None: ...


@dataclass(frozen=True)
class TeleoperationServoStatus:
    active: bool
    stopping: bool
    stop_reason: str | None
    error: str | None
    update_age_s: float
    processed_generation: int
    target_generation: int
    target_error: str | None
    target_joint_position: np.ndarray | None
    target_delta_xyz_m: np.ndarray | None
    target_orientation_xyzw: np.ndarray | None
    gripper_state: GripperState
    gripper_contact_latched: bool
    workspace_speed_scale: float
    cycle_period_s: float
    feedback_snapshot: ArmSnapshot | None


class TeleoperationServo:
    def __init__(
        self,
        controller: TeleoperationArm,
        *,
        config: TeleoperationConfig,
        profile_name: str,
        rate_hz: float,
        watchdog_timeout_s: float,
        state_stale_timeout_s: float,
        gripper_velocity_m_s: float,
        gripper_acceleration_m_s2: float,
        gripper_jerk_m_s3: float,
    ) -> None:
        if profile_name not in config.profiles:
            raise ValueError("unknown teleoperation profile")
        self._controller = controller
        self._config = config
        self._profile = config.profiles[profile_name]
        self._period_s = 1.0 / rate_hz
        self._watchdog_timeout_s = watchdog_timeout_s
        self._state_stale_timeout_s = state_stale_timeout_s
        self._gripper_limits = (
            gripper_velocity_m_s,
            gripper_acceleration_m_s2,
            gripper_jerk_m_s3,
        )
        self._condition = threading.Condition()
        self._latest_target = TeleoperationTarget(
            np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), GripperState.OPEN
        )
        self._update_generation = 0
        self._processed_generation = 0
        self._planned_generation = 0
        self._last_update_time = 0.0
        self._active = False
        self._stopping = False
        self._stop_requested = False
        self._stop_reason = None
        self._error = None
        self._target_error = None
        self._thread = None
```

Implement `start`, `update_target`, `wait_for_target`, `status`, and `stop` with the
same lock/join pattern already used by `CartesianServo`. `start` must reject a current
TCP outside `config.workspace`, snapshot the pose/joints/gripper, and start one daemon
thread. `update_target` must replace the latest target, increment generation, and update
the monotonic watchdog timestamp.

- [ ] **Step 7: Implement the 100 Hz loop**

Use one joint `JointRuckigPlanner` and one one-axis gripper `JointRuckigPlanner`. Initialize
`last_accepted_target` to zero translation, identity rotation, and `OPEN`; initialize
`last_target_process_time` from `time.monotonic()`. The target-processing branch must be:

```python
now = time.monotonic()
requested_position = start_pose[:3] + target.delta_xyz_m
if not self._config.workspace.contains(requested_position):
    raise SafetyError("teleoperation target is outside the configured workspace")
bounded_target = limit_teleoperation_target_step(
    last_accepted_target,
    target,
    self._profile,
    now - last_target_process_time,
)
target_position = start_pose[:3] + bounded_target.delta_xyz_m
workspace_scale = self._config.workspace.speed_scale(target_position)
target_orientation = quaternion_multiply_xyzw(
    bounded_target.delta_orientation_xyzw,
    start_orientation,
)
plan = self._controller.plan_cartesian_servo_target(
    start_pose,
    start_joint,
    bounded_target.delta_xyz_m,
    seed,
    target_orientation_xyzw=target_orientation,
)
arm_planner.set_limit_scale(
    min(workspace_scale, float(plan.singularity_speed_scale))
)
arm_planner.set_target(plan.target_joint_position)
seed = plan.target_joint_position.copy()
last_accepted_target = bounded_target
last_target_process_time = now
```

Import `limit_teleoperation_target_step` and `quaternion_multiply_xyzw` from
`arx5_control.teleoperation`. A rejected target must leave `last_accepted_target` and
`last_target_process_time` unchanged, so the next valid target is limited from the last
accepted state.

For gripper behavior:

- `OPEN` clears the contact latch and sets a position target at `open_width_m`.
- `CLOSE` sets a negative velocity target unless contact is already latched.
- Absolute gripper torque at or above `closing_torque_nm` brakes and latches contact.
- Approaching zero width uses the same stopping-distance guard as `GripperServo`; reaching
  that travel guard brakes and latches the gripper without stopping arm tracking.
- A new `OPEN` command releases the latch.
- Each cycle calls `command_joint_servo(arm.position, arm.velocity, gripper.position)` once.
- `cycle_period_s` records the latest difference between consecutive loop-start
  monotonic timestamps for browser telemetry.
- Stop/watchdog brakes both planners and exits only after both report zero-speed completion.
- Feedback timestamp staleness and any worker exception follow existing emergency-damping behavior.

- [ ] **Step 8: Export types and run tests**

Export `TeleoperationServo`, `TeleoperationServoStatus`, `TeleoperationTarget`, and
`GripperState` from `arx5_control/__init__.py`.

Run:

```bash
.venv/bin/python -m unittest tests.test_teleoperation tests.test_servo tests.test_controller_basics -v
```

Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add arx5_control/teleoperation.py arx5_control/teleoperation_servo.py tests/test_teleoperation.py
git add -p arx5_control/__init__.py
git diff --cached --check
git commit -m "feat(control): add unified teleoperation servo"
```

### Task 6: WebSocket Teleoperation Protocol

**Files:**
- Modify: `scripts/web_control_server.py`
- Modify: `tests/test_web_control.py`

- [ ] **Step 1: Add failing parser and protocol tests**

Import `teleoperation_target_from_params` in `tests/test_web_control.py` and add:

```python
def test_teleoperation_target_parser_validates_relative_pose(self) -> None:
    target = teleoperation_target_from_params({
        "delta_xyz_m": [0.01, -0.02, 0.03],
        "delta_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "gripper_state": "close",
    })
    np.testing.assert_allclose(target.delta_xyz_m, [0.01, -0.02, 0.03])
    self.assertEqual(target.gripper_state.value, "close")
    with self.assertRaisesRegex(ValueError, "quaternion"):
        teleoperation_target_from_params({
            "delta_xyz_m": [0.0, 0.0, 0.0],
            "delta_orientation_xyzw": [0.0, 0.0, 0.0, 0.0],
            "gripper_state": "open",
        })
```

Add this protocol test:

```python
def test_teleoperation_protocol_owns_session_and_rejects_wrong_id(self) -> None:
    class FakeWebSocket:
        def __init__(self) -> None:
            self.messages = []

        async def send(self, raw: str) -> None:
            self.messages.append(json.loads(raw))

    class FakeTeleoperationServo:
        instances = []

        def __init__(self, controller, **kwargs) -> None:
            self.controller = controller
            self.active = False
            self.generation = 0
            self.target = None
            FakeTeleoperationServo.instances.append(self)

        def start(self, **kwargs):
            self.active = True
            return self.controller.get_state()

        def update_target(self, target):
            self.generation += 1
            self.target = target
            return self.generation

        def wait_for_target(self, generation, timeout_s):
            return self.status()

        def stop(self, reason, *, emergency=False, timeout_s=3.0):
            self.active = False

        def status(self):
            joints = self.controller.joint_position.copy()
            return SimpleNamespace(
                active=self.active,
                stopping=False,
                error=None,
                stop_reason=None,
                update_age_s=0.0,
                processed_generation=self.generation,
                target_generation=self.generation,
                target_error=None,
                target_joint_position=joints if self.target else None,
                target_delta_xyz_m=(
                    None if self.target is None else self.target.delta_xyz_m
                ),
                target_orientation_xyzw=(
                    None if self.target is None
                    else self.target.delta_orientation_xyzw
                ),
                gripper_state=(
                    GripperState.OPEN if self.target is None
                    else self.target.gripper_state
                ),
                gripper_contact_latched=False,
                workspace_speed_scale=1.0,
                cycle_period_s=0.01,
                feedback_snapshot=None,
            )

    websocket = FakeWebSocket()
    self.session.controller.mode = ControllerMode.DAMPING  # type: ignore[union-attr]
    original_get_state = self.session.controller.get_state  # type: ignore[union-attr]

    def get_state_inside_workspace():
        snapshot = original_get_state()
        snapshot.eef_pose_6d = np.array([0.30, 0.0, 0.20, 0.0, 0.0, 0.0])
        return snapshot

    self.session.controller.get_state = get_state_inside_workspace  # type: ignore[method-assign,union-attr]

    async def scenario() -> None:
        await self.session.handle_teleoperation_message(websocket, {
            "type": "teleop", "seq": 1, "action": "start",
            "params": {"profile": "fine"},
        })
        session_id = websocket.messages[-1]["session_id"]
        target = {
            "delta_xyz_m": [0.01, 0.0, 0.0],
            "delta_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "gripper_state": "open",
        }
        await self.session.handle_teleoperation_message(websocket, {
            "type": "teleop", "seq": 2, "action": "update",
            "session_id": session_id, "params": target,
        })
        await self.session.handle_teleoperation_message(websocket, {
            "type": "teleop", "seq": 3, "action": "update",
            "session_id": "wrong-session", "params": target,
        })
        await self.session.handle_teleoperation_message(websocket, {
            "type": "teleop", "seq": 4, "action": "stop",
            "session_id": session_id, "params": {},
        })

    with patch(
        "scripts.web_control_server.TeleoperationServo",
        FakeTeleoperationServo,
    ):
        asyncio.run(scenario())

    self.assertEqual(
        [(message["action"], message["ok"]) for message in websocket.messages],
        [("start", True), ("update", True), ("update", False), ("stop", True)],
    )
    self.assertEqual(websocket.messages[0]["state"]["servo_kind"], "teleoperation")
    self.assertIsNone(self.session.teleoperation_session_id)

    def preview_plan(start_pose, start_joint, delta, seed, *, target_orientation_xyzw):
        target_pose = np.asarray(start_pose).copy()
        target_pose[:3] += np.asarray(delta)
        target_pose[3:] = quaternion_xyzw_to_rpy(target_orientation_xyzw)
        target_joint = np.asarray(start_joint).copy()
        target_joint[:3] += np.asarray(delta)
        return CartesianPlan(
            start_pose_6d=np.asarray(start_pose).copy(),
            target_pose_6d=target_pose,
            solved_pose_6d=target_pose.copy(),
            start_joint_position=np.asarray(start_joint).copy(),
            target_joint_position=target_joint,
            position_error_m=0.0,
            orientation_error_rad=0.0,
        )

    self.session.controller.plan_cartesian_servo_target = preview_plan  # type: ignore[attr-defined,method-assign,union-attr]
    preview_websocket = FakeWebSocket()

    async def preview_scenario() -> None:
        await self.session.handle_teleoperation_message(preview_websocket, {
            "type": "teleop", "seq": 5, "action": "start",
            "params": {"profile": "fine", "preview_only": True},
        })
        session_id = preview_websocket.messages[-1]["session_id"]
        await asyncio.sleep(0.02)
        await self.session.handle_teleoperation_message(preview_websocket, {
            "type": "teleop", "seq": 6, "action": "update",
            "session_id": session_id,
            "params": {
                "delta_xyz_m": [0.01, 0.0, 0.0],
                "delta_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "gripper_state": "open",
            },
        })
        await self.session.handle_teleoperation_message(preview_websocket, {
            "type": "teleop", "seq": 7, "action": "stop",
            "session_id": session_id, "params": {},
        })

    with patch(
        "scripts.web_control_server.TeleoperationServo",
        FakeTeleoperationServo,
    ):
        asyncio.run(preview_scenario())
    self.assertEqual(len(FakeTeleoperationServo.instances), 1)
    self.assertTrue(preview_websocket.messages[1]["preview"]["reachable"])
    self.assertEqual(
        preview_websocket.messages[0]["state"]["servo_kind"],
        "teleoperation_preview",
    )
```

Also import `GripperState` from `arx5_control` in this test file.

- [ ] **Step 2: Run the new protocol tests**

Run:

```bash
.venv/bin/python -m unittest \
  tests.test_web_control.WebControlTests.test_teleoperation_target_parser_validates_relative_pose \
  tests.test_web_control.WebControlTests.test_teleoperation_protocol_owns_session_and_rejects_wrong_id \
  -v
```

Expected: FAIL because the parser and protocol handler are missing.

- [ ] **Step 3: Add config loading and parser**

In `scripts/web_control_server.py`:

```python
import secrets

from arx5_control import (
    GripperState,
    TeleoperationConfig,
    TeleoperationServo,
    TeleoperationTarget,
    load_teleoperation_config,
)

TELEOPERATION_CONFIG_PATH = Path("configs/teleoperation.json")


def teleoperation_target_from_params(params: dict[str, Any]) -> TeleoperationTarget:
    if not isinstance(params, dict):
        raise ValueError("遥操作 params 必须是对象。")
    try:
        return TeleoperationTarget(
            delta_xyz_m=np.asarray(params["delta_xyz_m"], dtype=np.float64),
            delta_orientation_xyzw=np.asarray(
                params["delta_orientation_xyzw"], dtype=np.float64
            ),
            gripper_state=GripperState(params["gripper_state"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"遥操作目标无效：{exc}") from exc
```

Append `teleoperation_config: TeleoperationConfig | None = None` to the current
`WebArmSession.__init__` signature, then assign:

```python
self.teleoperation_config = (
    load_teleoperation_config(TELEOPERATION_CONFIG_PATH)
    if teleoperation_config is None else teleoperation_config
)
```

Load the config once in `main` and inject it into
`WebArmSession(..., teleoperation_config=config)`; the optional constructor default keeps
existing focused tests concise while still allowing in-memory config injection.

- [ ] **Step 4: Add session ownership and state**

Add these fields:

```python
self.teleoperation_servo: TeleoperationServo | None = None
self.teleoperation_sequence = -1
self.teleoperation_session_id: str | None = None
self.teleoperation_profile_name: str | None = None
self.teleoperation_last_message_at = 0.0
self.teleoperation_preview_only = False
self.teleoperation_preview_start_pose: np.ndarray | None = None
self.teleoperation_preview_start_joint: np.ndarray | None = None
self.teleoperation_preview_seed: np.ndarray | None = None
self.teleoperation_preview_target: TeleoperationTarget | None = None
self.teleoperation_preview_processed_at = 0.0
self.teleoperation_preview_generation = 0
self.teleoperation_preview_last_payload: dict[str, Any] | None = None
```

Add `teleoperation_session_active` as `self.teleoperation_session_id is not None`.
Preview-only sessions are not motor-busy and must not set `servo_active`, but they still
own the teleoperation protocol and are mutually exclusive with calibration, motion, and
the two existing servos.

Read its status beside the two existing servo statuses:

```python
teleoperation_status = (
    None
    if self.teleoperation_servo is None
    else self.teleoperation_servo.status()
)
```

Include it in `servo_active` and `servo_stopping`, include
`teleoperation_config.web_payload()` in connected and disconnected payloads, and replace
the existing `servo_kind` expression with:

```python
"servo_kind": (
    "teleoperation"
    if teleoperation_status and teleoperation_status.active
    else "teleoperation_preview"
    if self.teleoperation_preview_only and self.teleoperation_session_id is not None
    else "gripper"
    if gripper_servo_status and gripper_servo_status.active
    else "cartesian"
    if servo_status and servo_status.active
    else None
),
```

Also return:

```python
"teleoperation_config": self.teleoperation_config.web_payload(),
"teleoperation": {
    "active": bool(teleoperation_status and teleoperation_status.active),
    "preview_only": bool(
        self.teleoperation_preview_only
        and self.teleoperation_session_id is not None
    ),
    "stopping": bool(teleoperation_status and teleoperation_status.stopping),
    "target_age_ms": (
        1000.0 * teleoperation_status.update_age_s
        if teleoperation_status
        else 1000.0 * max(0.0, time.monotonic() - self.teleoperation_last_message_at)
        if self.teleoperation_preview_only and self.teleoperation_last_message_at > 0.0
        else None
    ),
    "servo_period_ms": (
        1000.0 * teleoperation_status.cycle_period_s
        if teleoperation_status else None
    ),
    "gripper_contact_latched": bool(
        teleoperation_status and teleoperation_status.gripper_contact_latched
    ),
},
```

- [ ] **Step 5: Implement `handle_teleoperation_message`**

Route `message["type"] == "teleop"` before command handling. Enforce integer increasing
`seq`, actions `start/update/stop`, no active motion/calibration/other servo, a current
TCP inside the workspace, matching `session_id`, and at most 60 update messages per
second using server `time.monotonic()`.

For `start`:

```python
profile_name = str(params.get("profile", "fine"))
profile = self.teleoperation_config.profiles[profile_name]
preview_only = params.get("preview_only", False)
if not isinstance(preview_only, bool):
    raise ValueError("preview_only 必须是布尔值。")
control_side = params.get("control_side", "right")
if control_side not in {"left", "right"}:
    raise ValueError("控制手必须是 left 或 right。")
open_width_mm = params.get("open_width_mm", 40.0)
closing_torque_nm = params.get("closing_torque_nm", 0.15)
if not finite_number(open_width_mm) or not 0.0 <= float(open_width_mm) <= 88.0:
    raise ValueError("夹爪张开宽度必须在 0..88 mm。")
if not finite_number(closing_torque_nm) or not 0.05 <= float(closing_torque_nm) <= 0.25:
    raise ValueError("夹爪闭合扭矩必须在 0.05..0.25 N·m。")
current = await asyncio.to_thread(controller.get_state)
if not self.teleoperation_config.workspace.contains(current.eef_pose_6d[:3]):
    raise SafetyError("当前 TCP 位于遥操作安全盒之外。")
if preview_only:
    self.teleoperation_session_id = secrets.token_urlsafe(18)
    self.teleoperation_profile_name = profile_name
    self.teleoperation_preview_only = True
    self.teleoperation_preview_start_pose = current.eef_pose_6d.copy()
    self.teleoperation_preview_start_joint = current.joint_position.copy()
    self.teleoperation_preview_seed = current.joint_position.copy()
    self.teleoperation_preview_target = TeleoperationTarget(
        np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), GripperState.OPEN
    )
    self.teleoperation_preview_processed_at = time.monotonic()
    self.teleoperation_last_message_at = self.teleoperation_preview_processed_at
    self.teleoperation_preview_generation = 0
    self.teleoperation_preview_last_payload = None
    await self.send_teleoperation_state(
        websocket,
        sequence,
        action,
        True,
        session_id=self.teleoperation_session_id,
    )
    return
controller.safety = replace(
    WEB_SAFE_MOTION_SAFETY,
    max_commanded_velocity_rad_s=max(0.35, profile.joint_velocity_rad_s),
    max_commanded_cartesian_velocity_m_s=profile.tcp_linear_velocity_m_s,
    feedback_velocity_limit_rad_s=max(0.60, 1.5 * profile.joint_velocity_rad_s),
    servo_max_velocity_rad_s=profile.joint_velocity_rad_s,
    servo_max_acceleration_rad_s2=profile.joint_acceleration_rad_s2,
    servo_max_jerk_rad_s3=profile.joint_jerk_rad_s3,
)
if controller.mode == ControllerMode.DAMPING:
    self.remember(await asyncio.to_thread(controller.hold_current))
servo = TeleoperationServo(
    controller,
    config=self.teleoperation_config,
    profile_name=profile_name,
    rate_hz=controller.safety.servo_rate_hz,
    watchdog_timeout_s=controller.safety.servo_watchdog_timeout_s,
    state_stale_timeout_s=controller.safety.state_stale_timeout_s,
    gripper_velocity_m_s=controller.safety.max_commanded_gripper_velocity_m_s,
    gripper_acceleration_m_s2=controller.safety.gripper_servo_max_acceleration_m_s2,
    gripper_jerk_m_s3=controller.safety.gripper_servo_max_jerk_m_s3,
)
snapshot = await asyncio.to_thread(
    servo.start,
    open_width_m=float(open_width_mm) / 1000.0,
    closing_torque_nm=float(closing_torque_nm),
)
self.teleoperation_session_id = secrets.token_urlsafe(18)
self.teleoperation_profile_name = profile_name
```

For `update`, parse the target, call `update_target`, wait up to 45 ms, and return a
preview containing target joints, relative XYZ, target quaternion, workspace scale,
gripper state, reachability, and error. For `stop`, brake normally and clear the ID.

When `self.teleoperation_preview_only` is true, do not create or call a servo. Process
the update synchronously with the same server target cap and planner:

```python
now = time.monotonic()
target = teleoperation_target_from_params(params)
if self.teleoperation_profile_name is None:
    raise ControlError("遥操作档位状态缺失。")
profile = self.teleoperation_config.profiles[self.teleoperation_profile_name]
start_pose = self.teleoperation_preview_start_pose
start_joint = self.teleoperation_preview_start_joint
seed = self.teleoperation_preview_seed
previous = self.teleoperation_preview_target
if any(value is None for value in (start_pose, start_joint, seed, previous)):
    raise ControlError("只读遥操作预览状态不完整。")
if now - self.teleoperation_last_message_at > 0.20:
    raise ControlError("只读遥操作预览已超时，请重新接通离合。")
requested_position = start_pose[:3] + target.delta_xyz_m
if not self.teleoperation_config.workspace.contains(requested_position):
    raise SafetyError("遥操作目标位于安全盒之外。")
bounded = limit_teleoperation_target_step(
    previous,
    target,
    profile,
    now - self.teleoperation_preview_processed_at,
)
target_position = start_pose[:3] + bounded.delta_xyz_m
workspace_scale = self.teleoperation_config.workspace.speed_scale(target_position)
target_orientation = quaternion_multiply_xyzw(
    bounded.delta_orientation_xyzw,
    rpy_to_quaternion_xyzw(start_pose[3:]),
)
plan = await asyncio.to_thread(
    controller.plan_cartesian_servo_target,
    start_pose,
    start_joint,
    bounded.delta_xyz_m,
    seed,
    target_orientation_xyzw=target_orientation,
)
self.teleoperation_preview_generation += 1
self.teleoperation_preview_target = bounded
self.teleoperation_preview_seed = plan.target_joint_position.copy()
self.teleoperation_preview_processed_at = now
self.teleoperation_last_message_at = now
preview = {
    "generation": self.teleoperation_preview_generation,
    "target_generation": self.teleoperation_preview_generation,
    "reachable": True,
    "target_joint_position_deg": np.degrees(plan.target_joint_position).tolist(),
    "delta_xyz_mm": (1000.0 * bounded.delta_xyz_m).tolist(),
    "target_orientation_xyzw": target_orientation.tolist(),
    "workspace_speed_scale": min(
        workspace_scale, float(plan.singularity_speed_scale)
    ),
    "gripper_state": bounded.gripper_state.value,
}
self.teleoperation_preview_last_payload = preview
```

If workspace/IK/collision validation rejects a preview update, return `ok: false` with
the previous accepted preview copied as `reachable: false` plus `error`; do not mutate
the seed, bounded target, generation, or processed timestamp. This supplies a safe
limit-colored Ghost without any motor command.

Use a dedicated response shape so the browser never confuses this stream with the
existing Cartesian/gripper servo protocol:

```python
async def send_teleoperation_state(
    self,
    websocket: ServerConnection,
    sequence: Any,
    action: str,
    ok: bool,
    *,
    session_id: str | None,
    error: str | None = None,
    preview: dict[str, Any] | None = None,
) -> None:
    payload: dict[str, Any] = {
        "type": "teleop_state",
        "seq": sequence,
        "action": action,
        "ok": ok,
        "session_id": session_id,
        "state": self.state_payload(),
    }
    if error is not None:
        payload["error"] = error
    if preview is not None:
        payload["preview"] = preview
    await self.send(websocket, payload)
```

Record a valid sequence before processing it. Reject every duplicate/out-of-order
message with `ok: false`; reject updates arriving less than `1 / 60` seconds after the
last accepted update without calling `update_target`. Echo the active session ID on
start/update failures, and echo the stopped ID in the stop response before clearing the
stored ID.

Centralize cleanup so preview-only state cannot survive a mode switch or disconnect:

```python
def clear_teleoperation_session(self) -> None:
    self.teleoperation_session_id = None
    self.teleoperation_profile_name = None
    self.teleoperation_preview_only = False
    self.teleoperation_preview_start_pose = None
    self.teleoperation_preview_start_joint = None
    self.teleoperation_preview_seed = None
    self.teleoperation_preview_target = None
    self.teleoperation_preview_processed_at = 0.0
    self.teleoperation_preview_generation = 0
    self.teleoperation_preview_last_payload = None
```

- [ ] **Step 6: Integrate all stop paths**

Add `stop_teleoperation_servo(reason, emergency)` and call it from:

- webpage emergency;
- controller close;
- browser disconnect;
- explicit disconnect;
- any operation that already stops the two existing servos.

The stop helper must brake the real worker when present, clear preview-only sessions
when no worker exists, and always call `clear_teleoperation_session()` in `finally`.
Existing command/servo entry points must stop a preview-only session before proceeding,
and must reject starting concurrent motion while a real teleoperation servo is active.

Browser loss uses emergency behavior consistent with the existing server; normal clutch
release uses non-emergency Ruckig braking.

- [ ] **Step 7: Run server tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control tests.test_teleoperation -v
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add scripts/web_control_server.py tests/test_web_control.py
git commit -m "feat(web): add teleoperation protocol"
```

### Task 7: Browser Control-Socket Transport

**Files:**
- Modify: `web/manual_control.js`
- Modify: `tests/test_web_control.py`

- [ ] **Step 1: Add failing static integration assertions**

Add to the existing frontend test in `tests/test_web_control.py`:

```python
self.assertIn("export function sendTeleoperationMessage", manual_javascript)
self.assertIn('type: "teleop"', manual_javascript)
self.assertIn('message.type === "teleop_state"', manual_javascript)
self.assertIn('"arx5-teleop-state"', manual_javascript)
```

- [ ] **Step 2: Run the frontend test**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control.WebControlTests.test_frontend_speed_defaults_to_fifty_percent -v
```

Expected: FAIL on the first teleoperation assertion.

- [ ] **Step 3: Implement transport on the existing socket**

In `web/manual_control.js` add:

```javascript
let teleoperationSequence = 0;
let lastTeleoperationResponseSequence = -1;

export function sendTeleoperationMessage(
  action,
  params = {},
  sessionId = null,
) {
  if (!socketIsOpen() || !armState.connected) return null;
  if (action === "update" && controlSocket.bufferedAmount > 32 * 1024) return null;
  teleoperationSequence += 1;
  controlSocket.send(JSON.stringify({
    type: "teleop",
    seq: teleoperationSequence,
    action,
    ...(sessionId ? { session_id: sessionId } : {}),
    ...(action === "update" ? { client_time_ms: Date.now() } : {}),
    params,
  }));
  return teleoperationSequence;
}
```

In `handleControlMessage`:

```javascript
if (message.type === "teleop_state") {
  if (!Number.isInteger(message.seq)
      || message.seq <= lastTeleoperationResponseSequence) return;
  lastTeleoperationResponseSequence = message.seq;
  if (message.state) updateState(message.state);
  window.dispatchEvent(new CustomEvent("arx5-teleop-state", { detail: message }));
  if (!message.ok) setMessage(message.error ?? "手势遥操作已停止。", "error");
  return;
}
```

On socket close, dispatch a failed stopped event so the camera controller cannot retain
an old `session_id`. In `beforeunload`, send a teleoperation `stop` when
`armState.servo_kind` is `"teleoperation"` or `"teleoperation_preview"` before closing
the socket.

- [ ] **Step 4: Run web-control tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add web/manual_control.js tests/test_web_control.py
git commit -m "feat(web): send teleoperation targets"
```

### Task 8: Mac Camera and Browser Teleoperation State Machine

**Files:**
- Create: `web/hand_teleoperation.mjs`
- Modify: `web/app.js`
- Modify: `tests/js/hand_teleoperation_math.test.mjs`
- Modify: `tests/test_hand_teleoperation_js.py`

- [ ] **Step 1: Add failing pure tests for hand selection and pose extraction**

Extend the import list in `tests/js/hand_teleoperation_math.test.mjs` with
`handPoseFromLandmarks` and `selectControlAndClutchHands`, then add:

```javascript
test("hand roles require both confident opposite sides", () => {
  const right = openHand();
  const left = openHand().map(({ x, y, z }) => point(-x, y, z));
  const result = {
    landmarks: [left, right],
    worldLandmarks: [left, right],
    handedness: [
      [{ categoryName: "Left", score: 0.94 }],
      [{ categoryName: "Right", score: 0.96 }],
    ],
  };
  const selected = selectControlAndClutchHands(result, "right");
  assert.equal(selected.control.side, "right");
  assert.equal(selected.clutch.side, "left");
  assert.equal(
    selectControlAndClutchHands({
      landmarks: [right],
      worldLandmarks: [right],
      handedness: [[{ categoryName: "Right", score: 0.96 }]],
    }, "right"),
    null,
  );
});

test("hand pose mirrors camera X and returns finite geometry", () => {
  const landmarks = openHand();
  landmarks[0] = point(0.25, 0.40, 0.0);
  const pose = handPoseFromLandmarks({
    landmarks,
    worldLandmarks: openHand(),
    side: "right",
  });
  assert.equal(pose.wrist.x, 0.75);
  assert.equal(pose.wrist.y, 0.40);
  assert.ok(Number.isFinite(pose.scale) && pose.scale > 0);
  assert.ok(pose.quaternion.every(Number.isFinite));
});
```

Add a lightweight lifecycle guard to `tests/test_hand_teleoperation_js.py`:

```python
def test_browser_controller_declares_every_stop_path(self) -> None:
    root = Path(__file__).resolve().parents[1]
    source = (root / "web/hand_teleoperation.mjs").read_text(encoding="utf-8")
    for token in (
        "if (!panel)",
        "requestVideoFrameCallback",
        "visibilitychange",
        'addEventListener("ended"',
        '"arx5-teleop-state"',
        '"arx5-teleoperation-mode"',
        "lastValidPairAt",
        "pendingStartDeadline",
        "1 / 30",
    ):
        self.assertIn(token, source)
```

- [ ] **Step 2: Run Node tests**

Run:

```bash
node --test tests/js/hand_teleoperation_math.test.mjs
.venv/bin/python -m unittest tests.test_hand_teleoperation_js -v
```

Expected: FAIL because selection/extraction exports and the browser controller are missing.

- [ ] **Step 3: Implement hand selection and extraction**

Add:

```javascript
export function selectControlAndClutchHands(result, controlSide) {
  const hands = result.landmarks.map((landmarks, index) => ({
    landmarks,
    worldLandmarks: result.worldLandmarks[index],
    side: String(result.handedness[index]?.[0]?.categoryName ?? "").toLowerCase(),
    score: Number(result.handedness[index]?.[0]?.score ?? 0),
  }));
  const control = hands.find((hand) => hand.side === controlSide && hand.score >= 0.70);
  const clutch = hands.find((hand) => hand.side !== controlSide && hand.score >= 0.70);
  return control && clutch ? { control, clutch } : null;
}

export function handPoseFromLandmarks(hand) {
  return {
    wrist: { x: 1 - hand.landmarks[0].x, y: hand.landmarks[0].y },
    scale: palmScale(hand.landmarks),
    quaternion: palmFrameQuaternion(hand.worldLandmarks, hand.side),
  };
}
```

- [ ] **Step 4: Create a controller that safely no-ops without markup**

Create `web/hand_teleoperation.mjs`. Import MediaPipe, math exports,
`getArmState`, `sendTeleoperationMessage`, and `setControlMessage`. The module must:

```javascript
const WASM_ROOT = new URL("./assets/mediapipe/wasm/", document.baseURI).href;
const HAND_MODEL = new URL(
  "./weights/browser/mediapipe/hand_landmarker.task",
  document.baseURI,
).href;
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
  fine: Object.freeze({ maxTranslationStepM: 0.005, maxRotationStepRad: 3 * Math.PI / 180 }),
  fast: Object.freeze({ maxTranslationStepM: 0.015, maxRotationStepRad: 8 * Math.PI / 180 }),
});
```

- query `#handTeleoperationPanel` and return without side effects when absent;
- request Mac video with `CAMERA_CONSTRAINTS` only after the gesture tool is selected;
- create Hand Landmarker with GPU delegate, VIDEO mode, two hands, and 0.60 model thresholds;
- schedule with `requestVideoFrameCallback` and never process while another frame is active;
- keep states `disabled/preview/waiting/starting/tracking/braking/lost/fault`;
- while Hand mode is selected and the controller is connected, allow clutch-driven
  preview-only sessions even when the UI switch is off; motor commands are armed only
  when the switch is enabled and controller mode is active or damping;
- use `ClutchLatch`, `PinchLatch`, and `TeleoperationMapper`;
- send `start` after five open-palm frames, hold updates until successful start response,
  send at no more than 30 Hz, and send `stop` on release/loss;
- store `lastValidPairAt` and `pendingStartDeadline`; treat either hand missing,
  confidence below 0.70, or the last valid pair older than 100 ms as loss, and time out
  a pending start after 500 ms;
- build `TeleoperationMapper` at engage time from the server-provided axis matrix and
  profile gains plus `BROWSER_PROFILE_LIMITS[this.profile]`;
- when the enable switch changes, stop the current preview/real session, clear the Ghost,
  reset the clutch, and require five fresh open-palm frames before starting the new mode;
- track each sent sequence with `performance.now()` and render rolling visual FPS,
  inference time, response RTT, server target age, and servo period;
- stop on `visibilitychange`, camera-track `ended`, page mode switch, socket close, or fault;
- reset all latches and the session ID after every stop.

Use this exact lifecycle shape:

```javascript
async engage(controlHand, nowSeconds) {
  this.mapper.engage(handPoseFromLandmarks(controlHand), nowSeconds);
  this.pendingStartSequence = sendTeleoperationMessage("start", {
    profile: this.profile,
    control_side: this.controlSide,
    open_width_mm: this.openWidthMm,
    closing_torque_nm: this.closingTorqueNm,
    preview_only: !this.armed,
  });
  this.state = "starting";
}

sendTarget(controlHand, ratio, nowSeconds) {
  if (!this.sessionId || nowSeconds - this.lastSendAt < 1 / 30) return;
  const mapped = this.mapper.update(handPoseFromLandmarks(controlHand), nowSeconds);
  sendTeleoperationMessage("update", {
    ...mapped,
    gripper_state: this.pinch.update(ratio),
  }, this.sessionId);
  this.lastSendAt = nowSeconds;
}

disengage(reason) {
  if (this.sessionId) sendTeleoperationMessage("stop", {}, this.sessionId);
  this.sessionId = null;
  this.pendingStartSequence = null;
  this.clutch.reset();
  this.pinch.reset("open");
  this.state = this.armed ? "waiting" : "disabled";
  this.renderState(reason);
}
```

End the module, after the controller class declaration, with:

```javascript
const panel = document.querySelector("#handTeleoperationPanel");
if (!panel) {
  // Static pages can load the module without the teleoperation workspace.
} else {
  new HandTeleoperationController(panel).initialize();
}
```

- [ ] **Step 5: Pause the old D455 pose pipeline during hand mode**

In `web/app.js` listen for `arx5-teleoperation-mode`. When active, set the existing
pose-side switch to off and call the existing stop-recognition path without stopping the
D455 WebRTC camera.

- [ ] **Step 6: Run Node and Python web tests**

Run:

```bash
node --test tests/js/hand_teleoperation_math.test.mjs
.venv/bin/python -m unittest tests.test_hand_teleoperation_js tests.test_web_control -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add web/hand_teleoperation.mjs web/hand_teleoperation_math.mjs web/app.js tests/js/hand_teleoperation_math.test.mjs
git add tests/test_hand_teleoperation_js.py
git commit -m "feat(web): track Mac hand gestures"
```

### Task 9: Shared Gesture/Calibration Tool Card

**Files:**
- Modify: `web/index.html`
- Modify: `web/styles.css`
- Modify: `web/robot_viewer.js`
- Modify: `tests/test_web_control.py`

- [ ] **Step 1: Add failing markup assertions**

Assert all IDs and relationships:

```python
self.assertIn('data-mode="hand"', html)
self.assertIn('id="visionToolModes"', html)
self.assertIn('data-vision-tool="gesture"', html)
self.assertIn('data-vision-tool="handeye"', html)
self.assertIn('id="handTeleoperationPanel"', html)
self.assertIn('id="handCameraVideo"', html)
self.assertIn('id="handCameraOverlay"', html)
self.assertIn('id="teleoperationEnabled"', html)
self.assertIn('id="teleoperationProfile"', html)
self.assertIn('id="teleoperationControlSide"', html)
self.assertIn('id="teleoperationOpenWidth"', html)
self.assertIn('id="teleoperationClosingTorque"', html)
self.assertIn('id="teleoperationFps"', html)
self.assertIn('id="teleoperationInferenceMs"', html)
self.assertIn('id="teleoperationRttMs"', html)
self.assertIn('id="teleoperationTargetAgeMs"', html)
self.assertIn('id="teleoperationServoPeriodMs"', html)
self.assertIn('src="hand_teleoperation.mjs"', html)
self.assertLess(
    html.index('id="handTeleoperationPanel"'),
    html.index('id="handeyeCalibrationPanel"'),
)
```

- [ ] **Step 2: Run the markup test**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control.WebControlTests.test_frontend_speed_defaults_to_fifty_percent -v
```

Expected: FAIL on missing hand mode.

- [ ] **Step 3: Add the control mode and shared tool shell**

Add a third `data-mode="hand"` button to `#robotControlModes`. Add a hidden, status-only
`#handControlPane` in the robot inspector; it may show current teleoperation state but
must contain no duplicate controls.

Wrap the existing `.workspace` and `#handeyeCalibrationPanel` in a new
`.vision-tool-shell`. Insert the tool tabs before the unchanged shared D455 `.workspace`,
then insert this new gesture panel after that workspace:

```html
<div id="visionToolModes" class="binary-segment" data-selected="gesture"
     aria-label="视觉工具">
    <button type="button" data-vision-tool="gesture" class="selected"
            aria-pressed="true">
      <i class="fa-solid fa-hand" aria-hidden="true"></i>手势控制
    </button>
    <button type="button" data-vision-tool="handeye" aria-pressed="false">
      <i class="fa-solid fa-crosshairs" aria-hidden="true"></i>手眼标定
    </button>
</div>
<section id="handTeleoperationPanel" class="hand-teleoperation-panel">
    <div class="hand-camera-view">
      <video id="handCameraVideo" autoplay muted playsinline></video>
      <canvas id="handCameraOverlay"></canvas>
    </div>
    <div class="hand-teleoperation-controls">
      <label class="servo-toggle">
        <input id="teleoperationEnabled" type="checkbox" />
        <span><strong>遥操作</strong><small id="teleoperationState">未启用</small></span>
      </label>
      <div id="teleoperationProfile" class="binary-segment" data-selected="fine">
        <button type="button" data-profile="fine" class="selected">精细</button>
        <button type="button" data-profile="fast">快速</button>
      </div>
      <div id="teleoperationControlSide" class="binary-segment" data-selected="right">
        <button type="button" data-side="left">左手</button>
        <button type="button" data-side="right" class="selected">右手</button>
      </div>
      <label>张开 mm
        <input id="teleoperationOpenWidth" type="number" min="0" max="88"
               step="1" value="40" />
      </label>
      <label>闭合 N·m
        <input id="teleoperationClosingTorque" type="number" min="0.05"
               max="0.25" step="0.01" value="0.15" />
      </label>
      <dl class="hand-teleoperation-metrics">
        <div><dt>视觉</dt><dd id="teleoperationFps">—</dd></div>
        <div><dt>推理</dt><dd id="teleoperationInferenceMs">—</dd></div>
        <div><dt>RTT</dt><dd id="teleoperationRttMs">—</dd></div>
        <div><dt>目标年龄</dt><dd id="teleoperationTargetAgeMs">—</dd></div>
        <div><dt>伺服周期</dt><dd id="teleoperationServoPeriodMs">—</dd></div>
      </dl>
    </div>
</section>
```

Inside the outer shell, move the existing complete `#handeyeCalibrationPanel` node
immediately after `#handTeleoperationPanel`; do not edit its inner markup. Move the Mac
video/overlay into a camera view shown only for the gesture tool. Keep D455 RGB visible
and hide the depth pseudo-color view in gesture mode.

- [ ] **Step 4: Add deterministic tool-switch behavior**

In `hand_teleoperation.mjs`:

- selecting hand robot mode selects gesture tool;
- selecting hand-eye stops teleoperation, waits for a `stop` response or 500 ms watchdog
  margin, then reveals calibration;
- active calibration forces hand-eye selected and disables the gesture tab;
- leaving hand robot mode disarms teleoperation;
- page-hidden and camera-stop paths remain idempotent.

- [ ] **Step 5: Style stable desktop and mobile layouts**

Add the structural rules below, using the existing CSS variables for actual colors:

```css
.robot-control-modes {
  grid-template-columns: repeat(3, minmax(0, 1fr));
}

.vision-tool-shell {
  display: grid;
  gap: 12px;
  min-width: 0;
}

.vision-tool-shell > [hidden],
#handControlPane[hidden] {
  display: none !important;
}

.vision-tool-shell .handeye-calibration-overlay {
  position: static;
  inset: auto;
  min-height: 0;
  background: transparent;
}

.hand-teleoperation-panel {
  display: grid;
  grid-template-columns: minmax(0, 2fr) minmax(220px, 1fr);
  gap: 12px;
  min-width: 0;
}

.hand-camera-view {
  position: relative;
  min-block-size: 240px;
  aspect-ratio: 16 / 9;
  overflow: hidden;
  border-radius: 6px;
}

#handCameraVideo,
#handCameraOverlay {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  object-fit: cover;
}

#handCameraVideo {
  transform: scaleX(-1);
}

.hand-teleoperation-controls {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  align-content: start;
  gap: 10px;
  min-width: 0;
}

.hand-teleoperation-controls .servo-toggle,
.hand-teleoperation-controls .binary-segment {
  grid-column: 1 / -1;
}

.hand-teleoperation-controls .binary-segment button {
  min-width: 0;
  white-space: nowrap;
}

.hand-teleoperation-controls input[type="number"] {
  width: 100%;
  min-width: 0;
  box-sizing: border-box;
}

.hand-teleoperation-metrics {
  grid-column: 1 / -1;
  display: grid;
  grid-template-columns: repeat(5, minmax(0, 1fr));
  gap: 6px;
  margin: 0;
}

.hand-teleoperation-metrics div {
  min-width: 0;
}

.hand-teleoperation-metrics dd {
  margin: 0;
  overflow-wrap: anywhere;
}

@media (max-width: 720px) {
  .hand-teleoperation-panel {
    grid-template-columns: minmax(0, 1fr);
  }

  .hand-camera-view {
    min-block-size: 180px;
  }

  .hand-teleoperation-metrics {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
}
```

Wrap the video/canvas in `.hand-camera-view` and all controls in
`.hand-teleoperation-controls`. Keep the shell unframed and panels mutually exclusive;
on narrow screens stack the Mac panel below the shared D455 view rather than overlaying
the videos. Use existing neutral/green/warning/red tokens and add no gradient.

- [ ] **Step 6: Extend robot mode handling**

Change `CONTROL_MODES` to `["xyz", "joint", "hand"]`. In `setControlMode`:

```javascript
elements.xyzPane.hidden = mode !== "xyz";
elements.jointPane.hidden = mode !== "joint";
elements.handPane.hidden = mode !== "hand";
window.dispatchEvent(new CustomEvent("arx5-teleoperation-mode", {
  detail: { active: mode === "hand" },
}));
```

Disable the existing Cartesian mouse servo and gripper keyboard servo before entering
hand mode. Do not show TCP drag controls or the joint ring while hand mode is selected.

- [ ] **Step 7: Load the controller module and run web tests**

Add:

```html
<script type="module" src="hand_teleoperation.mjs"></script>
```

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control tests.test_hand_teleoperation_js -v
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add web/index.html web/styles.css web/robot_viewer.js web/hand_teleoperation.mjs tests/test_web_control.py
git commit -m "feat(web): add gesture control workspace"
```

### Task 10: Server-Validated Ghost and Safety Box

**Files:**
- Modify: `scripts/web_control_server.py`
- Modify: `web/manual_control.js`
- Modify: `web/robot_viewer.js`
- Modify: `tests/test_web_control.py`

- [ ] **Step 1: Add failing preview-payload assertions**

Extend the fake protocol test to require:

```python
preview = websocket.messages[1]["preview"]
self.assertEqual(preview["generation"], 1)
self.assertTrue(preview["reachable"])
self.assertEqual(len(preview["target_joint_position_deg"]), 6)
self.assertEqual(len(preview["target_orientation_xyzw"]), 4)
self.assertIn("workspace_speed_scale", preview)
self.assertEqual(preview["gripper_state"], "open")
```

Add frontend source assertions for `teleoperation-workspace-box` and
`arx5-teleop-state` in `robot_viewer.js`.

- [ ] **Step 2: Run protocol and frontend tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control -v
```

Expected: FAIL on missing preview fields or viewer listener.

- [ ] **Step 3: Return a complete preview from the server**

Build preview only from `TeleoperationServoStatus`:

```python
preview = {
    "generation": status.processed_generation,
    "target_generation": status.target_generation,
    "reachable": (
        status.target_error is None
        and status.target_generation >= generation
    ),
    "target_joint_position_deg": (
        None if status.target_joint_position is None
        else np.degrees(status.target_joint_position).tolist()
    ),
    "delta_xyz_mm": (
        None if status.target_delta_xyz_m is None
        else (1000.0 * status.target_delta_xyz_m).tolist()
    ),
    "target_orientation_xyzw": (
        None if status.target_orientation_xyzw is None
        else status.target_orientation_xyzw.tolist()
    ),
    "workspace_speed_scale": status.workspace_speed_scale,
    "gripper_state": status.gripper_state.value,
}
if status.target_error:
    preview["error"] = status.target_error
```

- [ ] **Step 4: Draw the server-owned safety box**

In `web/robot_viewer.js` create one `THREE.LineSegments` from
`EdgesGeometry(new THREE.BoxGeometry(size.x, size.y, size.z))`, name it
`"teleoperation-workspace-box"`, position it at the min/max midpoint, and show it only
in hand mode. Rebuild it when `armState.teleoperation_config.workspace` changes.

- [ ] **Step 5: Drive the Ghost only from accepted server preview**

Listen for `arx5-teleop-state`. For any update response with six finite,
server-derived joint values (including a rejected update carrying the last accepted
preview):

```javascript
previewKind = message.preview.reachable
  ? "teleoperation"
  : "teleoperation-limit";
previewDirty = true;
previewAngles = message.preview.target_joint_position_deg.map(Number);
targetGripperWidthMm = message.preview.gripper_state === "open"
  ? Number(document.querySelector("#teleoperationOpenWidth").value)
  : targetGripperWidthMm;
updatePreviewVisibility();
updateInspector();
```

On stop, socket loss, or a failure without a finite preview, clear only the teleoperation
Ghost and retain actual feedback. Use the existing limit color when `reachable` is false.
Never drive Ghost directly from unvalidated browser hand data. Preview-only sessions use
this same event path and never call a motor-command API.

- [ ] **Step 6: Run focused tests**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control tests.test_teleoperation -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add scripts/web_control_server.py web/manual_control.js web/robot_viewer.js tests/test_web_control.py
git commit -m "feat(web): preview safe teleoperation targets"
```

### Task 11: Documentation and Final Verification

**Files:**
- Modify: `README.md`
- Modify: `docs/MEDIAPIPE.md`
- Modify: `docs/SAFETY.md`
- Modify: `docs/WEB_CONSOLE.md`
- Test: all `tests/` and browser modules

- [ ] **Step 1: Update operator documentation**

Document these exact operator steps:

1. Start `./arx5 can-up` and `./arx5 web-tailnet`.
2. Open HTTPS on the M3 Max and grant Mac camera permission.
3. Connect the arm, select Hand mode, verify axis directions with Ghost, then enable.
4. Hold the non-control hand open for five frames; release it to stop.
5. Pinch the control hand to close; open the pinch to command 40 mm.
6. Start in Fine mode with an empty area; use Fast only after stop-path checks.

Add the default workspace, 200 ms watchdog, two speed profiles, torque range, direct-power
fall hazard, and the statement that software/web emergency does not replace physical
emergency stop.

- [ ] **Step 2: Run whitespace and placeholder checks**

Run:

```bash
git diff --check
git log -p --format= HEAD~10..HEAD -- arx5_control web scripts tests configs | rg '^\+.*(TBD|TODO|FIXME|implement later)'
git diff -U0 -- README.md docs | rg '^\+.*(TBD|TODO|FIXME|implement later)'
```

Expected: `git diff --check` exits 0; both added-line searches print nothing and exit 1.

- [ ] **Step 3: Run focused test groups**

Run:

```bash
.venv/bin/python -m unittest tests.test_hand_teleoperation_js -v
.venv/bin/python -m unittest tests.test_teleoperation -v
.venv/bin/python -m unittest tests.test_controller_basics tests.test_cartesian tests.test_servo -v
.venv/bin/python -m unittest tests.test_web_control -v
```

Expected: PASS for every command.

- [ ] **Step 4: Run the complete hardware-free suite**

Run:

```bash
./arx5 test
```

Expected: all tests PASS without opening CAN.

- [ ] **Step 5: Inspect the UI at desktop and mobile sizes**

Serve the static frontend:

```bash
.venv/bin/python -m http.server 8010 --directory web
```

Open `http://127.0.0.1:8010` in a browser and inspect at 1440x900 and 390x844:

- no overlapping controls or clipped labels;
- gesture/calibration switch shows exactly one tool;
- Mac preview remains 16:9;
- emergency button and status bar remain visible;
- Hand mode hides mouse TCP controls and joint controls;
- camera denial shows an error without arming.

The static server has no control WebSocket, so disconnected state is expected.

- [ ] **Step 6: Perform camera-only acceptance**

Start `./arx5 web-tailnet` without connecting the controller, grant the Mac camera, and
select Hand. Verify dual-hand overlay, 30 FPS target, side selection, open-palm state,
pinch state, and that no control WebSocket teleoperation session starts.

- [ ] **Step 7: Perform read-only Ghost acceptance**

Support the arm against gravity, connect it, explicitly enter DAMPING, select Hand, and
leave `#teleoperationEnabled` off. Engage the clutch and verify server-validated Ghost
translation/rotation directions, workspace rejection, and `servo_kind` equal to
`teleoperation_preview`. Confirm from logs/tests that no `command_joint_servo` call occurs.
Stop the preview before enabling real motion.

- [ ] **Step 8: Commit documentation**

```bash
git add docs/MEDIAPIPE.md docs/WEB_CONSOLE.md
git add -p README.md docs/SAFETY.md
git diff --cached --check
git commit -m "docs: document hand teleoperation"
```

- [ ] **Step 9: Review the final diff**

Run:

```bash
git status --short
git diff --stat HEAD~11..HEAD
git log -11 --oneline
```

Expected: eleven focused feature commits contain only planned hunks. Pre-existing user
changes may remain in `git status`; no planned new file or planned hunk remains uncommitted.
