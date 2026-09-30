# Motion Feedback Regressions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate false J6 acceleration failures and keep the Web 3D arm responsive to measured feedback throughout long motions.

**Architecture:** Treat a confirmed park trajectory as a stationary-start trajectory, so planning and execution validate the same velocity boundary. Make the controller motion monitor the single SDK feedback reader during motion and publish its latest measured snapshot to the Web session. Bound and throttle expensive Three.js path geometry rebuilds independently from the lightweight per-frame joint-angle interpolation.

**Tech Stack:** Python 3.12, `unittest`, NumPy, Ruckig, asyncio/websockets, Three.js ES modules.

---

### Task 1: Stationary Park Trajectory Boundary

**Files:**
- Modify: `tests/test_controller_basics.py`
- Modify: `arx5_control/controller.py`

- [ ] **Step 1: Write the failing regression test**

Add a controller test that plans a smooth joint preview while J6 reports `+0.04 rad/s`, executes it after J6 reports `-0.04 rad/s`, and validates the generated samples. Assert that execution supplies a zero `trajectory_start_velocity`; the current implementation omits it and reproduces `trajectory acceleration for joint 6 exceeds the configured limit`.

- [ ] **Step 2: Verify the test fails for the expected boundary mismatch**

Run:

```bash
.venv/bin/python -B -m unittest \
  tests.test_controller_basics.ControllerBasicsTests.test_smooth_preview_execution_uses_stationary_velocity_boundary
```

Expected: failure because `_run_motion` receives no stationary start velocity, or because J6 acceleration validation sees the feedback-velocity jump.

- [ ] **Step 3: Use one stationary boundary for preview and execution**

In `plan_smooth_joint_trajectory_preview`, require feedback velocity to be within the existing terminal stationary tolerance, then pass zero velocity and acceleration to Ruckig. In `execute_smooth_joint_trajectory_preview`, repeat the stationary check and call `_run_motion(..., trajectory_start_velocity=np.zeros_like(start.joint_velocity))`.

- [ ] **Step 4: Verify the targeted and controller suites pass**

Run:

```bash
.venv/bin/python -B -m unittest tests.test_controller_basics
```

Expected: all controller tests pass.

### Task 2: Single-Reader Motion Feedback Stream

**Files:**
- Modify: `tests/test_controller_basics.py`
- Modify: `tests/test_web_control.py`
- Modify: `arx5_control/controller.py`
- Modify: `scripts/web_control_server.py`

- [ ] **Step 1: Write failing cache and stream tests**

Add one controller test proving monitored joint feedback can be converted into a cached `ArmSnapshot` with FK-derived TCP pose. Extend the long-motion Web test with a controller whose live `get_state()` raises during the worker but whose cached snapshot advances; assert `motion_state` arrives before `command_result` without a concurrent live read.

- [ ] **Step 2: Verify both tests fail**

Run:

```bash
.venv/bin/python -B -m unittest \
  tests.test_controller_basics.ControllerBasicsTests.test_monitored_feedback_updates_cached_snapshot \
  tests.test_web_control.WebControlTests.test_long_motion_streams_feedback_before_result
```

Expected: missing cached-feedback API and/or an attempted live `get_state()` call.

- [ ] **Step 3: Publish monitor-owned snapshots**

Add a small lock-protected latest-snapshot cache to `ArmController`. Store validated full snapshots from `get_state()`, and during both arm and gripper monitor loops store snapshots assembled from the already-read joint feedback plus FK-derived `eef_pose_6d`. Expose a read-only `get_cached_state()` method that never calls the SDK.

- [ ] **Step 4: Consume only cached feedback during Web motion**

Change `WebArmSession.run_operation` to call `controller.get_cached_state()` for `motion_state` messages. Keep normal idle polling and final state reads unchanged, because no motion-monitor reader is active at those times.

- [ ] **Step 5: Verify feedback tests pass**

Run:

```bash
.venv/bin/python -B -m unittest tests.test_controller_basics tests.test_web_control
```

Expected: all tests pass and the long-motion test still observes feedback before the result.

### Task 3: Bounded Three.js Path Updates

**Files:**
- Modify: `tests/test_web_control.py`
- Modify: `web/robot_viewer.js`

- [ ] **Step 1: Add a failing source-level Web regression check**

Follow the repository's existing browser-source assertions: require explicit constants for a maximum executed-path point count and a minimum geometry rebuild interval, plus a bounded decimation path.

- [ ] **Step 2: Verify the Web test fails**

Run:

```bash
.venv/bin/python -B -m unittest \
  tests.test_web_control.WebControlTests.test_web_assets_include_robot_preview
```

Expected: the new path-bounding assertions are absent.

- [ ] **Step 3: Bound and throttle geometry allocation**

Keep every feedback update available to `handleArmState` and the lightweight joint-angle interpolation. Cap accumulated executed-path points by decimating older points while preserving the newest point, and rebuild tube/line geometry no more often than a short fixed interval unless a start/finish call forces it. Apply the same time gate to pending-path split geometry.

- [ ] **Step 4: Run all non-hardware tests and syntax checks**

Run:

```bash
./arx5 test
git diff --check
```

Expected: the complete test suite passes and Git reports no whitespace errors.

- [ ] **Step 5: Verify the browser at desktop and mobile sizes**

Start the local static server, load the page with Playwright, confirm the WebGL canvas is nonblank, and inspect browser console output. Exercise synthetic `arx5-state` updates to confirm joint state changes do not depend on path geometry refreshes.
