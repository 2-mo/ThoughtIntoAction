# Unified Motion Primitives Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Route point-to-point web and cube motion through controller-owned `movej` and `movep` primitives while replacing single-sample terminal velocity failures with measured-position settlement.

**Architecture:** `ArmController` owns target validation, planning, speed scaling, execution, and completion. Web and cube workflows supply only absolute targets plus the selected speed. Existing public methods remain compatibility wrappers while the new primitives become the entry points for new workflows.

**Tech Stack:** Python 3.12, NumPy, Ruckig, `unittest`, asyncio/websockets, browser ES modules.

---

### Task 1: Terminal Feedback Window

**Files:**
- Modify: `tests/test_controller_basics.py`
- Modify: `arx5_control/controller.py:62-214`
- Modify: `arx5_control/controller.py:2636-2719`

- [ ] **Step 1: Write failing settlement tests**

Add tests that exercise a pure controller settlement predicate with timestamped
position and velocity samples:

```python
def test_terminal_window_accepts_velocity_noise_when_position_is_stable(self):
    target = np.zeros(6)
    samples = (
        (10.00, np.zeros(6), np.asarray([0, 0, 0.066, 0, 0, 0])),
        (10.06, np.full(6, 0.0004), np.asarray([0, 0, 0.066, 0, 0, 0])),
        (10.12, np.full(6, 0.0002), np.asarray([0, 0, 0.066, 0, 0, 0])),
    )
    self.assertTrue(controller._terminal_feedback_is_settled(samples, target))

def test_terminal_window_rejects_real_position_drift(self):
    samples = (
        (10.00, np.zeros(6), np.zeros(6)),
        (10.06, np.asarray([0, 0, 0.004, 0, 0, 0]), np.zeros(6)),
        (10.12, np.asarray([0, 0, 0.008, 0, 0, 0]), np.zeros(6)),
    )
    self.assertFalse(controller._terminal_feedback_is_settled(samples, np.zeros(6)))
```

- [ ] **Step 2: Verify RED**

Run:

```bash
./arx5 test tests/test_controller_basics.py -k terminal_window
```

Expected: both tests fail because `_terminal_feedback_is_settled` does not
exist.

- [ ] **Step 3: Add explicit settlement configuration**

Add to `SafetyConfig` and its positive-value validation:

```python
terminal_stability_window_s: float = 0.12
terminal_position_stability_rad: float = math.radians(0.15)
```

Reject a stability window greater than `settle_time_s`.

- [ ] **Step 4: Implement the window predicate**

Add a private method that requires a full time window, checks every sample is
within target position tolerance, then accepts either low reported velocity or
position span below `terminal_position_stability_rad`:

```python
def _terminal_feedback_is_settled(self, samples, target):
    if len(samples) < 2:
        return False
    if samples[-1][0] - samples[0][0] < self.safety.terminal_stability_window_s:
        return False
    positions = np.stack([sample[1] for sample in samples])
    velocities = np.stack([sample[2] for sample in samples])
    target_error = np.max(np.abs(positions - target), axis=0)
    position_span = np.ptp(positions, axis=0)
    return bool(
        np.all(target_error <= self.safety.terminal_position_tolerance_rad)
        and np.max(position_span)
        <= self.safety.terminal_position_stability_rad
    )
```

- [ ] **Step 5: Make `_monitor_motion` condition-based**

Retain samples from `finish_timestamp` onward in a `collections.deque`. Trim
samples older than the stability window, return once the predicate is true,
and otherwise continue only until the existing `settle_time_s` deadline. On
failure report final target error, reported velocity, and measured position
span. Keep all existing stale feedback, torque, tracking, and wall-clock checks.

- [ ] **Step 6: Verify GREEN**

Run:

```bash
./arx5 test tests/test_controller_basics.py
```

Expected: the controller suite passes, including the `0.066 rad/s` regression.

### Task 2: Canonical `movej` and `movep`

**Files:**
- Modify: `tests/test_controller_basics.py`
- Modify: `arx5_control/controller.py:1005-1132`
- Modify: `arx5_control/controller.py:1154-1280`
- Modify: `arx5_control/controller.py:1650-1874`

- [ ] **Step 1: Write failing primitive tests**

Add controller tests proving:

```python
preview = controller.plan_movej(target, velocity_scale=0.25)
self.assertAlmostEqual(preview.duration_s, canonical_duration / 0.25)

controller.movep(
    np.asarray([0.30, -0.10, 0.25]),
    np.asarray([0.0, 0.0, 0.0, 1.0]),
    velocity_scale=1.0,
    path_mode="linear",
    unrestricted=True,
)
```

Patch the existing execution methods in the second test and assert `movep`
converts the absolute target to one relative delta and does not accept a caller
duration.

- [ ] **Step 2: Verify RED**

Run:

```bash
./arx5 test tests/test_controller_basics.py -k 'movej or movep'
```

Expected: failures report missing `plan_movej`, `movej`, and `movep` methods.

- [ ] **Step 3: Implement joint primitives**

Implement `plan_movej` as the canonical derivative scaling policy and `movej`
as plan plus exact preview execution:

```python
def plan_movej(self, target, *, velocity_scale=1.0, allow_large_motion=False):
    return self.plan_smooth_joint_trajectory_preview(
        target,
        velocity_scale=velocity_scale,
        acceleration_scale=velocity_scale**2,
        jerk_scale=velocity_scale**3,
        allow_large_motion=allow_large_motion,
    )

def movej(self, target, *, velocity_scale=1.0, allow_large_motion=False):
    preview = self.plan_movej(
        target,
        velocity_scale=velocity_scale,
        allow_large_motion=allow_large_motion,
    )
    return self.execute_smooth_joint_trajectory_preview(
        preview,
        allow_large_motion=allow_large_motion,
    )
```

- [ ] **Step 4: Implement Cartesian primitives**

Implement `plan_movep` and `movep` with absolute XYZ plus quaternion targets.
Both validate finite `(3,)` and `(4,)` arrays, derive the relative delta from
measured feedback, and delegate `linear` to the Cartesian waypoint kernel and
`smooth` to the joint-space Cartesian curve kernel. No duration argument is
exposed.

- [ ] **Step 5: Make linear previews use the waypoint kernel**

In `plan_cartesian_trajectory`, replace the old smoothstep-derived linear
duration branch with `_build_cartesian_geometry_from_start` plus
`_prepare_cartesian_geometry_trajectory`, then adapt the result to the existing
`CartesianTrajectoryPreview` response type. Keep smooth preview semantics
unchanged for compatibility.

- [ ] **Step 6: Verify GREEN**

Run:

```bash
./arx5 test tests/test_controller_basics.py
```

Expected: all controller primitive and existing compatibility tests pass.

### Task 3: Migrate Web and Cube Workflows

**Files:**
- Modify: `tests/test_web_control.py`
- Modify: `scripts/web_control_server.py:651-719`
- Modify: `scripts/web_control_server.py:3279-3368`
- Modify: `web/manual_control.js:690-702`

- [ ] **Step 1: Replace the old cube-duration test with failing routing tests**

Assert that `_cube_pick_execute` calls `controller.movep` with the absolute
target and stored velocity scale, while `plan_cartesian_trajectory` is patched
to raise if called:

```python
with patch.object(controller, "plan_cartesian_trajectory", side_effect=AssertionError):
    with patch.object(controller, "movep", wraps=controller.movep) as movep:
        self.session._cube_pick_execute(controller, (action,), config)
movep.assert_called_once()
self.assertEqual(movep.call_args.kwargs["velocity_scale"], 1.0)
```

Add a source-contract assertion that ordinary XYZ execution contains
`controller.movep(` and the automation delay is `100` rather than `1200`.

- [ ] **Step 2: Verify RED**

Run:

```bash
./arx5 test tests/test_web_control.py -k 'cube or frontend or cartesian'
```

Expected: cube routing and frontend-delay assertions fail against the legacy
duration prepass and `1200 ms` timeout.

- [ ] **Step 3: Route cube actions through `movep`**

Replace the cube `move_to` preplan plus relative move with:

```python
controller.movep(
    np.asarray(target_xyz_m, dtype=np.float64),
    np.asarray(orientation_xyzw, dtype=np.float64),
    velocity_scale=config.motion.velocity_scale,
    path_mode="linear",
    unrestricted=True,
)
```

Remove the obsolete cube-specific Cartesian duration floor import from the web
server. Preserve force-close and gripper behavior.

- [ ] **Step 4: Route ordinary XYZ through `movep`**

After the existing preview-lock checks, derive the absolute target from the
stored preview and call `controller.movep` with the selected `path_mode` and
speed. Keep the response and browser protocol unchanged.

- [ ] **Step 5: Shorten only automatic preview dwell**

Change the `holdCubeStackPreview` timeout from `1200` to `100`. Manual previews
remain confirmation-gated.

- [ ] **Step 6: Verify GREEN**

Run:

```bash
./arx5 test tests/test_web_control.py
node --experimental-default-type=module --check web/manual_control.js
```

Expected: web tests and JavaScript syntax checks pass.

### Task 4: Full Verification and Runtime Reload

**Files:**
- Verify only; no new production files.

- [ ] **Step 1: Run all offline checks**

```bash
./arx5 test
node --experimental-default-type=module --check web/manual_control.js
node --experimental-default-type=module --check web/app.js
node --experimental-default-type=module --check web/robot_viewer.js
git diff --check
```

Expected: all Python tests pass with only optional OpenCV skips, all JavaScript
files parse, and Git reports no whitespace errors.

- [ ] **Step 2: Restart the web service**

Gracefully stop the current `web-lan` session, start `./arx5 web-lan` with the
same bind/IP environment, and confirm ports `8000`, `8765`, `8766`, and `8767`
are listening.

- [ ] **Step 3: Verify the live protocol**

Connect to the control WebSocket, check the motion profile revision handshake,
and confirm independent state responses continue while no physical motion is
requested.

- [ ] **Step 4: Report hardware-test boundary**

Report offline and live protocol evidence separately. Do not claim physical
cube motion is verified unless the user runs an actual pick-and-stack cycle.
