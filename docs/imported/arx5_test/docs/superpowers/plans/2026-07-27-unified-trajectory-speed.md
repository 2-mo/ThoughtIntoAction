# Unified Cartesian Trajectory Speed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Cartesian motion use one server-trusted safety profile, generate one speed-independent geometric/IK path, and execute every speed tier as a smooth time-scaled and interpolated version of that path.

**Architecture:** Add reusable Hermite time-scaling helpers. Cartesian planning creates canonical samples at 100%, validates them, then uses C2 quintic-Hermite interpolation to derive scaled timestamps and dense command samples without another IK solve. The web layer always installs one safety profile and keeps `speed_percent` only as a time-scale input.

**Implementation refinement:** The initial cubic helper remains as a compatibility primitive. Final Cartesian execution uses `resample_quintic_hermite` with position, velocity, and acceleration anchors because C1 cubic segments can introduce acceleration discontinuities and finite-difference jerk spikes at IK anchors.

**Tech Stack:** Python 3, NumPy, Ruckig, ARX X5 SDK, `unittest`, browser JavaScript/HTML/CSS.

---

### Task 1: Add continuous trajectory resampling

**Files:**
- Create: `arx5_control/motion/interpolation.py`
- Modify: `arx5_control/motion/__init__.py`
- Test: `tests/test_trajectory.py`

- [ ] **Step 1: Write the failing tests**

Add tests for `resample_cubic_hermite(times, positions, velocities, *, speed_scale, sample_rate_hz)`. A one-second move at `0.25` must return `0.0..4.0` at 20 Hz (81 samples), preserve endpoint positions/velocities, and have strictly increasing times. A three-anchor move must reproduce the middle position and velocity at the exact scaled anchor time.

- [ ] **Step 2: Run the focused tests and verify RED**

Run: `./arx5 test tests/test_trajectory.py -k resample_cubic_hermite`

Expected: collection fails because the new module/helper does not exist.

- [ ] **Step 3: Implement the helper**

Validate finite shapes, strictly increasing canonical times, positive `speed_scale` and `sample_rate_hz`, and matching joint dimensions. Generate `ceil(times[-1] / speed_scale * sample_rate_hz)` intervals capped at 4000, always retaining the exact final sample. Evaluate each segment with cubic Hermite basis functions and their analytic velocity derivative:

```python
h00 = 2*u**3 - 3*u**2 + 1
h10 = u**3 - 2*u**2 + u
h01 = -2*u**3 + 3*u**2
h11 = u**3 - u**2
position = h00*p0 + h10*dt*v0 + h01*p1 + h11*dt*v1
velocity = ((6*u**2 - 6*u)*p0
          + (3*u**2 - 4*u + 1)*dt*v0
          + (-6*u**2 + 6*u)*p1
          + (3*u**2 - 2*u)*dt*v1) / dt
```

Scale canonical query times by `1/speed_scale` and canonical velocities by `speed_scale`. Explicitly set exact endpoints and exact anchor queries to avoid segment-boundary roundoff.

- [ ] **Step 4: Export and verify**

Export the helper from `arx5_control/motion/__init__.py`. Run `./arx5 test tests/test_trajectory.py -k resample_cubic_hermite`; expected: all new tests pass.

- [ ] **Step 5: Commit**

Run `git add arx5_control/motion/interpolation.py arx5_control/motion/__init__.py tests/test_trajectory.py && git commit -m "feat: add smooth trajectory resampling"`.

### Task 2: Make Cartesian planning canonical and speed-independent

**Files:**
- Modify: `arx5_control/controller.py:370,1273-1375,1556-1735,1780-1810`
- Modify: `arx5_control/motion/limits.py`
- Test: `tests/test_controller_basics.py`, `tests/test_trajectory.py`

- [ ] **Step 1: Write failing tests**

Add controller tests that plan the same Cartesian target at scales `0.25`, `0.5`, and `1.0`. Assert identical `joint_positions`, more resampled command points at lower scale, duration ratios near `4:2:1`, and a rejection above 300 seconds. Preserve the regression test that a moving Cartesian start is rejected while the command boundary uses zero start velocity.

- [ ] **Step 2: Run RED**

Run: `./arx5 test tests/test_controller_basics.py -k canonical_path`. Expected: the current implementation changes IK sample count and derivative limits with `velocity_scale`, so the canonical-path assertions fail.

- [ ] **Step 3: Generate one canonical geometry/IK trajectory**

In `_prepare_cartesian_geometry_trajectory` and `move_cartesian_relative`, call `plan_scalar_progress` with unscaled Cartesian limits and `minimum_motion_duration_s`. For an explicit final duration, use `max(minimum_motion_duration_s, duration_s * velocity_scale)` as the canonical minimum. Solve IK once from the current seed. Run TCP, singularity, and joint dynamics checks on this canonical path; do not call `_scaled_derivative_limits(velocity_scale)` while constructing it.

- [ ] **Step 4: Time-scale and interpolate the exact path**

After the canonical dynamic checks, derive acceleration anchors and use `resample_quintic_hermite(...)` to produce one C2-continuous dense trajectory. Validate the 100% dense arrays, stretch the canonical timestamps if required, then apply the requested `velocity_scale`. Use the returned arrays, excluding the initial row, to build `TrajectorySample` values. Keep the final zero-velocity hold. Validate these exact samples through `_run_motion` with `trajectory_start_velocity=np.zeros_like(start.joint_velocity)`. Set `EXTENDED_MOTION_MAX_DURATION_S = 300.0`; keep the normal 30-second cap.

- [ ] **Step 5: Apply the same rule to smooth XYZ preview**

Plan smooth XYZ Ruckig at 100% once, then resample its positions/velocities for the requested speed. Make `move_cartesian_relative_smooth` consume the same canonical representation. Preserve `linear`/`smooth` labels and existing preview payload fields.

- [ ] **Step 6: Run focused tests and commit**

Run `./arx5 test tests/test_trajectory.py tests/test_controller_basics.py`; expected: focused tests pass, including the nonzero-feedback Cartesian-boundary regression. Then run `git add arx5_control/controller.py arx5_control/motion/limits.py tests/test_controller_basics.py tests/test_trajectory.py && git commit -m "fix: time-scale one Cartesian trajectory"`.

### Task 3: Collapse web motion profiles

**Files:**
- Modify: `scripts/web_control_server.py:78-195,300-345,2210-2240,2380-2400`
- Modify: `web/manual_control.js:15-145,1750-1785`
- Modify: `web/index.html:150-172`
- Test: `tests/test_web_control.py`

- [ ] **Step 1: Write failing contract tests**

Assert `motion_safety_from_params({})` and `motion_safety_from_params({"speed_percent": 25})` are the same `WEB_MOTION_SAFETY`. Assert supplied legacy `speed_profile`/`hardware_max_confirmed` fields are rejected, hello limits contain no profile or hardware-max fields, HTML has no `hardwareMaxSpeedButton`, and JavaScript speed params contain only `speed_percent`.

- [ ] **Step 2: Run RED**

Run `./arx5 test tests/test_web_control.py -k 'speed or frontend'`; expected: current profile/toggle assertions fail.

- [ ] **Step 3: Implement one server profile**

Delete the safe/hardware profile constants, hardware safety object, and hardware park-margin branch. Keep `WEB_MOTION_SAFETY = WEB_SAFE_MOTION_SAFETY`. Make `motion_safety_from_params` reject legacy profile fields and return the singleton. Remove profile and hardware velocity fields from the hello payload; continue validating `speed_percent`.

- [ ] **Step 4: Remove the browser toggle**

Remove the hardware-max button markup, query, helper, listeners, and label mutation. `getMotionSpeedParams()` must return exactly `{speed_percent: getMotionSpeedPercent()}`. Keep the ten existing speed tiers.

- [ ] **Step 5: Run and commit**

Run `./arx5 test tests/test_web_control.py`; expected: web tests pass. Then run `git add scripts/web_control_server.py web/manual_control.js web/index.html tests/test_web_control.py && git commit -m "fix: use one web motion safety profile"`.

### Task 4: Align documentation and duration/error reporting

**Files:** `docs/SAFETY.md`, `docs/TRAJECTORY_CONTROL.md`, `docs/WEB_CONSOLE.md`, `web/README.md`, `tests/test_web_control.py`

- [ ] **Step 1: Add documentation contract assertions**

Assert docs describe speed percentages as time scaling and interpolation under one hard envelope, mention the 300-second extended limit, and no longer describe a hardware-max toggle.

- [ ] **Step 2: Update docs**

Document canonical 100% planning, `1/s` timestamps, `s` velocities, fixed-rate Hermite resampling, zero Cartesian start velocity, 300-second extended duration, and opt-in link capsules. Remove instructions for confirming or enabling SDK hardware maximums.

- [ ] **Step 3: Run and commit**

Run `./arx5 test tests/test_web_control.py`; expected: all documentation contract checks and web tests pass. Then run `git add docs/SAFETY.md docs/TRAJECTORY_CONTROL.md docs/WEB_CONSOLE.md web/README.md tests/test_web_control.py && git commit -m "docs: describe unified trajectory speed"`.

### Task 5: Full verification and safe service handoff

**Files:** no planned changes; edit only a failing implementation/test file if verification identifies a regression.

- [ ] **Step 1: Run static checks and full tests**

Run `git diff --check` and `./arx5 test`; expected: `OK` with no diff-check output.

- [ ] **Step 2: Run the no-motion cube preview diagnostic**

For cube `(398, 42, 15) mm`, verify 25%, 50%, and 100% use identical joint positions, 25% takes roughly four times 100%, and no motor command is submitted. Reject only if the computed duration exceeds 300 seconds.

- [ ] **Step 3: Check the running service without restarting hardware**

Confirm `runtime/web-lan.pid` still points to a process whose CWD is `/home/r2/Desktop/arx5_test`. Do not restart automatically. In the final handoff instruct the user to ensure the arm is damped/stopped, stop the old server in its original terminal, then run `cd /home/r2/Desktop/arx5_test && ./arx5 web-lan`.

- [ ] **Step 4: Review the final diff**

Run `git status --short` and `git diff --stat`; ensure unrelated dirty files were neither staged nor reverted, and report tests plus the required server restart.
