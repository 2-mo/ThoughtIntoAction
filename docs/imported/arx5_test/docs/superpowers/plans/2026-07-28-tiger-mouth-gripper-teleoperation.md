# Tiger-Mouth Gripper Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Use a stable tiger-mouth local frame for binary gripper teleoperation and smooth relative TCP orientation with balanced latency.

**Architecture:** Keep landmark geometry and temporal state in `hand_teleoperation_math.mjs`. A dedicated orientation tracker derives the thumb-index opening axis and tiger-mouth plane normal, holds the last valid frame for 100 ms during geometric degeneration, and falls back to the palm frame only at initialization. The existing browser controller owns and resets that tracker; server protocol and safety enforcement remain unchanged.

**Tech Stack:** Browser ES modules, MediaPipe Hand Landmarker world landmarks, quaternion SLERP, Node.js built-in test runner, Python `unittest` wrapper.

---

## File Map

- Modify `web/hand_teleoperation_math.mjs`: tiger-mouth basis construction, degeneracy tracker, pose integration, 80 ms rotation smoothing default.
- Modify `web/hand_teleoperation.mjs`: create/reset tracker and pass timestamps into pose extraction.
- Modify `tests/js/hand_teleoperation_math.test.mjs`: deterministic geometry, hysteresis, fallback/expiry, smoothing, and integration tests.
- Verify `tests/test_hand_teleoperation_js.py` and `tests/test_web_control.py`: existing JavaScript wrapper and unchanged WebSocket contract.

### Task 1: Tiger-Mouth Geometry And Degeneracy Tracking

**Files:**
- Modify: `web/hand_teleoperation_math.mjs:45-115`
- Test: `tests/js/hand_teleoperation_math.test.mjs:4-97`

- [ ] **Step 1: Add deterministic tiger-mouth fixtures and failing tests**

Add `TigerMouthOrientationTracker` and `tigerMouthFrameQuaternion` to the test imports. Add this fixture after `openHand()`:

```javascript
function tigerMouthHand() {
  const hand = openHand();
  hand[2] = point(-0.5, 0, 0);
  hand[5] = point(0.5, 0, 0);
  hand[4] = point(-1, 1, 0);
  hand[8] = point(1, 1, 0);
  return hand;
}
```

Add these tests before the clutch tests:

```javascript
test('tiger-mouth frame uses opening, longitudinal, and plane-normal axes', () => {
  const quaternion = tigerMouthFrameQuaternion(tigerMouthHand(), 'right');
  assert.ok(quaternionDistanceRadians(quaternion, [0, 0, 0, 1]) < 1e-12);
});

test('tiger-mouth frame normalizes left handedness casing', () => {
  assert.deepEqual(
    tigerMouthFrameQuaternion(tigerMouthHand(), 'Left'),
    tigerMouthFrameQuaternion(tigerMouthHand(), 'left'),
  );
});

test('tiger-mouth tracker holds a closed frame then expires', () => {
  const tracker = new TigerMouthOrientationTracker({ holdSeconds: 0.1 });
  const valid = tracker.update(tigerMouthHand(), 'right', 1);
  const closed = tigerMouthHand();
  closed[8] = { ...closed[4] };

  assert.deepEqual(tracker.update(closed, 'right', 1.05), valid);
  assert.throws(() => tracker.update(closed, 'right', 1.11), /degenerate/);
});

test('tiger-mouth tracker falls back to palm frame only at initialization', () => {
  const tracker = new TigerMouthOrientationTracker({ holdSeconds: 0.1 });
  const closed = tigerMouthHand();
  closed[8] = { ...closed[4] };

  const fallback = tracker.update(closed, 'right', 2);
  assert.ok(quaternionDistanceRadians(
    fallback,
    palmFrameQuaternion(closed, 'right'),
  ) < 1e-12);
  assert.throws(() => tracker.update(closed, 'right', 2.11), /degenerate/);
});

test('tiger-mouth tracker requires increasing finite timestamps', () => {
  const tracker = new TigerMouthOrientationTracker();
  tracker.update(tigerMouthHand(), 'right', 3);
  assert.throws(() => tracker.update(tigerMouthHand(), 'right', 3), /timestamp/);
  assert.throws(
    () => new TigerMouthOrientationTracker().update(tigerMouthHand(), 'right', Number.NaN),
    /timestamp/,
  );
});
```

- [ ] **Step 2: Run focused tests and verify missing exports**

Run:

```bash
node --test --test-name-pattern='tiger-mouth' tests/js/hand_teleoperation_math.test.mjs
```

Expected: FAIL because `TigerMouthOrientationTracker` and `tigerMouthFrameQuaternion` are not exported.

- [ ] **Step 3: Implement tiger-mouth frame construction**

Add after `palmFrameQuaternion()`:

```javascript
function midpoint(first, second) {
  return {
    x: (first.x + second.x) / 2,
    y: (first.y + second.y) / 2,
    z: (first.z + second.z) / 2,
  };
}

export function tigerMouthFrameQuaternion(hand, handedness) {
  if (!Array.isArray(hand) || hand.length !== 21) {
    throw new TypeError('tiger-mouth frame requires 21 landmarks');
  }
  const opening = normalize(vector(hand[4], hand[8]));
  const root = midpoint(hand[2], hand[5]);
  const tips = midpoint(hand[4], hand[8]);
  const longitudinalRaw = normalize(vector(root, tips));
  const projection = dot(longitudinalRaw, opening);
  const longitudinal = normalize(longitudinalRaw.map(
    (value, index) => value - projection * opening[index],
  ));
  let normal = normalize(cross(opening, longitudinal));
  if (String(handedness).toLowerCase() === 'left') {
    normal = normal.map((value) => -value);
  }
  const canonicalLongitudinal = normalize(cross(normal, opening));
  return matrixQuaternion([
    [opening[0], canonicalLongitudinal[0], normal[0]],
    [opening[1], canonicalLongitudinal[1], normal[1]],
    [opening[2], canonicalLongitudinal[2], normal[2]],
  ]);
}
```

- [ ] **Step 4: Implement bounded degeneracy tracking**

Add after `closestQuaternion()`:

```javascript
export class TigerMouthOrientationTracker {
  constructor({ holdSeconds = 0.1 } = {}) {
    this.holdSeconds = holdSeconds;
    this.reset();
  }

  reset() {
    this.quaternion = null;
    this.lastValidTimestamp = null;
    this.timestamp = null;
  }

  update(hand, handedness, timestamp) {
    if (!Number.isFinite(timestamp)) {
      throw new TypeError('orientation timestamp must be finite');
    }
    if (this.timestamp !== null && timestamp <= this.timestamp) {
      throw new RangeError('orientation timestamp must increase');
    }
    this.timestamp = timestamp;
    try {
      const candidate = tigerMouthFrameQuaternion(hand, handedness);
      this.quaternion = this.quaternion
        ? closestQuaternion(candidate, this.quaternion)
        : candidate;
      this.lastValidTimestamp = timestamp;
      return this.quaternion.slice();
    } catch (error) {
      if (!(error instanceof RangeError)) throw error;
      if (this.quaternion === null) {
        this.quaternion = palmFrameQuaternion(hand, handedness);
        this.lastValidTimestamp = timestamp;
        return this.quaternion.slice();
      }
      if (timestamp - this.lastValidTimestamp <= this.holdSeconds) {
        return this.quaternion.slice();
      }
      throw new RangeError('tiger-mouth frame remained degenerate');
    }
  }
}
```

- [ ] **Step 5: Run the focused tests**

Run:

```bash
node --test --test-name-pattern='tiger-mouth' tests/js/hand_teleoperation_math.test.mjs
```

Expected: 5 tiger-mouth tests PASS; unrelated tests are skipped by the name filter.

- [ ] **Step 6: Commit geometry and tests**

```bash
git add web/hand_teleoperation_math.mjs tests/js/hand_teleoperation_math.test.mjs
git commit -m "feat(web): derive tiger-mouth orientation"
```

### Task 2: Wire The Tracker Into Browser Teleoperation

**Files:**
- Modify: `web/hand_teleoperation_math.mjs:426-432`
- Modify: `web/hand_teleoperation.mjs:6-15,91-94,440-479,516-540`
- Test: `tests/js/hand_teleoperation_math.test.mjs:233-245`

- [ ] **Step 1: Add failing pose integration tests**

Replace the final hand-pose test with:

```javascript
test('hand pose mirrors camera X and uses tracked tiger-mouth orientation', () => {
  const landmarks = tigerMouthHand();
  landmarks[0] = point(0.25, 0.40, 0.0);
  const tracker = new TigerMouthOrientationTracker();
  const pose = handPoseFromLandmarks({
    landmarks,
    worldLandmarks: tigerMouthHand(),
    side: 'right',
  }, tracker, 1);

  assert.equal(pose.wrist.x, 0.75);
  assert.equal(pose.wrist.y, 0.40);
  assert.ok(Number.isFinite(pose.scale) && pose.scale > 0);
  assert.ok(quaternionDistanceRadians(pose.quaternion, [0, 0, 0, 1]) < 1e-12);
});

test('hand pose requires a timestamp when orientation tracking is enabled', () => {
  assert.throws(() => handPoseFromLandmarks({
    landmarks: tigerMouthHand(),
    worldLandmarks: tigerMouthHand(),
    side: 'right',
  }, new TigerMouthOrientationTracker()), /timestamp/);
});
```

- [ ] **Step 2: Run tests and verify the pose still ignores the tracker**

Run:

```bash
node --test --test-name-pattern='hand pose' tests/js/hand_teleoperation_math.test.mjs
```

Expected: FAIL because the tracker is not called and the missing timestamp is not rejected.

- [ ] **Step 3: Extend pose extraction without changing its output contract**

Replace `handPoseFromLandmarks()` with:

```javascript
export function handPoseFromLandmarks(hand, orientationTracker = null, timestamp = null) {
  const quaternion = orientationTracker
    ? orientationTracker.update(hand.worldLandmarks, hand.side, timestamp)
    : tigerMouthFrameQuaternion(hand.worldLandmarks, hand.side);
  return {
    wrist: { x: 1 - hand.landmarks[0].x, y: hand.landmarks[0].y },
    scale: palmScale(hand.landmarks),
    quaternion,
  };
}
```

- [ ] **Step 4: Give the browser controller one tracker per teleoperation lifecycle**

Import `TigerMouthOrientationTracker` in `web/hand_teleoperation.mjs`. In the constructor, after `this.pinch`, add:

```javascript
this.orientationTracker = new TigerMouthOrientationTracker();
```

Change both pose calls:

```javascript
this.mapper.engage(
  handPoseFromLandmarks(controlHand, this.orientationTracker, nowSeconds),
  nowSeconds,
);
```

```javascript
const mapped = this.mapper.update(
  handPoseFromLandmarks(controlHand, this.orientationTracker, nowSeconds),
  nowSeconds,
);
```

Reset temporal orientation state alongside the existing clutch/pinch resets in the `stop`/`socket_closed` handler and `disengage()`:

```javascript
this.orientationTracker.reset();
```

- [ ] **Step 5: Run JavaScript teleoperation tests**

Run:

```bash
node --test tests/js/hand_teleoperation_math.test.mjs
```

Expected: all tests PASS.

- [ ] **Step 6: Commit browser integration**

```bash
git add web/hand_teleoperation.mjs web/hand_teleoperation_math.mjs tests/js/hand_teleoperation_math.test.mjs
git commit -m "feat(web): track tiger-mouth control frame"
```

### Task 3: Balanced Rotation Smoothing And Regression Verification

**Files:**
- Modify: `web/hand_teleoperation_math.mjs:305-384`
- Test: `tests/js/hand_teleoperation_math.test.mjs:99-196`
- Verify: `tests/test_hand_teleoperation_js.py`
- Verify: `tests/test_web_control.py`

- [ ] **Step 1: Add a failing 80 ms smoothing test**

Add after the mapper origin test:

```javascript
test('mapper uses an 80 ms balanced rotation time constant', () => {
  const mapper = new TeleoperationMapper({
    operatorToBase: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    translationGain: 1,
    rotationGain: 1,
    maxRotationStepRad: Math.PI,
  });
  mapper.engage(
    { wrist: { x: 0.5, y: 0.5 }, scale: 0.2, quaternion: [0, 0, 0, 1] },
    0,
  );
  const target = mapper.update({
    wrist: { x: 0.5, y: 0.5 },
    scale: 0.2,
    quaternion: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
  }, 0.08);
  const actual = quaternionDistanceRadians(
    [0, 0, 0, 1],
    target.delta_orientation_xyzw,
  );
  const expected = Math.PI / 2 * (1 - Math.exp(-1));
  assert.ok(Math.abs(actual - expected) < 1e-12);
});
```

- [ ] **Step 2: Run the focused test and verify the old 50 ms default fails**

Run:

```bash
node --test --test-name-pattern='80 ms' tests/js/hand_teleoperation_math.test.mjs
```

Expected: FAIL because default `rotationTimeConstantS` is `0.05`.

- [ ] **Step 3: Set the balanced smoothing default**

In `TeleoperationMapper` constructor parameters, change:

```javascript
rotationTimeConstantS = 0.08,
```

Keep existing shortest-arc SLERP, 1.5-degree dead zone, and profile-specific maximum rotation step unchanged.

- [ ] **Step 4: Run focused and wrapper tests**

Run:

```bash
node --test tests/js/hand_teleoperation_math.test.mjs
.venv/bin/python -m unittest tests.test_hand_teleoperation_js -v
```

Expected: all Node tests PASS and Python wrapper PASS.

- [ ] **Step 5: Run WebSocket contract and static checks**

Run:

```bash
.venv/bin/python -m unittest tests.test_web_control -v
git diff --check
```

Expected: Web control tests PASS; `git diff --check` prints no output.

- [ ] **Step 6: Commit smoothing**

```bash
git add web/hand_teleoperation_math.mjs tests/js/hand_teleoperation_math.test.mjs
git commit -m "perf(web): smooth tiger-mouth rotation"
```

### Task 4: Final Feature Verification

**Files:**
- Verify: `web/hand_teleoperation_math.mjs`
- Verify: `web/hand_teleoperation.mjs`
- Verify: `tests/js/hand_teleoperation_math.test.mjs`

- [ ] **Step 1: Run the complete relevant test set**

```bash
.venv/bin/python -m unittest \
  tests.test_hand_teleoperation_js \
  tests.test_web_control \
  tests.test_teleoperation -v
```

Expected: all tests PASS without hardware.

- [ ] **Step 2: Inspect only task-related changes**

```bash
git status --short
git diff main...HEAD -- \
  web/hand_teleoperation_math.mjs \
  web/hand_teleoperation.mjs \
  tests/js/hand_teleoperation_math.test.mjs \
  docs/superpowers/specs/2026-07-28-tiger-mouth-gripper-teleoperation-design.md \
  docs/superpowers/plans/2026-07-28-tiger-mouth-gripper-teleoperation.md
```

Expected: no unrelated paths in the feature diff; no uncommitted task changes.

- [ ] **Step 3: Perform staged hardware validation**

With an independent physical emergency stop ready, use the fine profile only:

1. Preview Ghost orientation and confirm thumb-to-index motion matches the gripper opening axis.
2. Confirm tiger-mouth plane normal moves the gripper approach direction without sudden flips.
3. Close the tiger mouth for at least three frames and confirm binary close; reopen past hysteresis for three frames and confirm binary open.
4. Hold the tiger mouth nearly closed and confirm orientation remains stable for brief landmark degeneration.
5. Release the clutch and confirm the server brakes and stops within its existing watchdog behavior.

Expected: direction mapping is correct, visible jitter is reduced, measured response remains usable, and all safety stops remain active.
