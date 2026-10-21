# Hardware-Based Speed Tiers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the single web motion profile use the X5 hardware velocity ceilings at 100%, scale every lower tier from that baseline, and default the UI and protocol to 10%.

**Architecture:** Keep the existing `speed_percent`-only protocol and one `SafetyConfig`. Replace its conservative limits with the former hardware-profile derivative values, using a scalar 5.5 rad/s application ceiling that is intersected with `RobotConfig.joint_vel_max`; update browser and server defaults together.

**Tech Stack:** Python 3.12, NumPy, ARX5 SDK configuration, browser JavaScript/HTML, `unittest`.

---

### Task 1: Lock the Hardware Baseline and 10% Default

**Files:**
- Modify: `tests/test_web_control.py:970-1035`
- Test: `tests/test_web_control.py`

- [ ] **Step 1: Change the existing speed contract test before production code**

Assert the default parser result and full-speed configuration:

```python
self.assertEqual(velocity_scale_from_params({}), 0.10)
self.assertEqual(WEB_DEFAULT_SPEED_PERCENT, 10.0)
self.assertEqual(WEB_MOTION_SAFETY.max_commanded_velocity_rad_s, 5.5)
self.assertEqual(WEB_MOTION_SAFETY.max_commanded_cartesian_velocity_m_s, 0.60)
self.assertEqual(WEB_MOTION_SAFETY.max_commanded_gripper_velocity_m_s, 0.30)
self.assertEqual(WEB_MOTION_SAFETY.servo_max_acceleration_rad_s2, 4.0)
self.assertEqual(WEB_MOTION_SAFETY.servo_max_jerk_rad_s3, 24.0)
self.assertEqual(WEB_MOTION_SAFETY.minimum_motion_duration_s, 0.25)
```

Update the frontend contract assertions to require `value="5"` and `>10%</output>`.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `./arx5 test tests/test_web_control.py -k 'speed or frontend'`

Expected: failures report the current 50% default, 0.25 rad/s joint cap, 0.02 m/s Cartesian cap, and conservative derivative values.

### Task 2: Install the Full-Speed Profile

**Files:**
- Modify: `scripts/web_control_server.py:74-165`
- Modify: `web/manual_control.js:105-125`
- Modify: `web/index.html:153-166`
- Test: `tests/test_web_control.py`

- [ ] **Step 1: Replace the single web profile values**

Set `WEB_DEFAULT_SPEED_PERCENT = 10.0` and define `WEB_MOTION_SAFETY` with:

```python
WEB_MOTION_SAFETY = replace(
    SafetyConfig(),
    max_commanded_velocity_rad_s=5.5,
    max_commanded_cartesian_velocity_m_s=0.60,
    max_commanded_cartesian_acceleration_m_s2=1.20,
    max_commanded_cartesian_jerk_m_s3=12.0,
    feedback_velocity_limit_rad_s=5.75,
    max_commanded_gripper_velocity_m_s=0.30,
    feedback_gripper_velocity_limit_m_s=0.36,
    gripper_servo_max_acceleration_m_s2=1.20,
    gripper_servo_max_jerk_m_s3=6.0,
    trajectory_rate_hz=100.0,
    minimum_motion_duration_s=0.25,
    settle_time_s=0.65,
    trajectory_final_hold_s=0.30,
    terminal_position_tolerance_rad=math.radians(2.0),
    terminal_velocity_tolerance_rad_s=0.04,
    servo_max_velocity_rad_s=5.5,
    servo_max_acceleration_rad_s2=4.0,
    servo_max_jerk_rad_s3=24.0,
)
```

Keep `WEB_SAFE_MOTION_SAFETY = WEB_MOTION_SAFETY` only as a compatibility alias for existing imports; do not reintroduce request profiles.

- [ ] **Step 2: Align browser defaults**

Change the slider index from `value="7"` to `value="5"`, its output from `50%` to `10%`, and both JavaScript fallback branches from 50 to 10:

```javascript
if (!Number.isFinite(value)) return MOTION_SPEED_TIERS.indexOf(10);
return MOTION_SPEED_TIERS[getMotionSpeedIndex()] ?? 10;
```

- [ ] **Step 3: Run focused tests and verify GREEN**

Run: `./arx5 test tests/test_web_control.py -k 'speed or frontend'`

Expected: all selected web speed and frontend contract tests pass.

### Task 3: Update Operator Documentation and Verify

**Files:**
- Modify: `docs/SAFETY.md:5-15`
- Modify: `docs/WEB_CONSOLE.md:60-72`
- Modify: `web/README.md:132-145`
- Test: `tests/test_web_control.py`

- [ ] **Step 1: Update the documented semantics**

Document that 100% means SDK per-joint ceilings `[5, 5, 5.5, 5.5, 5, 5] rad/s`, TCP 0.6 m/s, and gripper 0.3 m/s; lower tiers time-scale that baseline and the initial tier is 10%. Remove statements that all tiers use the old 0.25 rad/s conservative envelope.

- [ ] **Step 2: Run static checks**

Run:

```bash
git diff --check
node --check --input-type=module < web/manual_control.js
```

Expected: both commands exit zero with no output.

- [ ] **Step 3: Run full offline verification**

Run: `./arx5 test`

Expected: 188 tests pass with only the existing optional OpenCV skips.

- [ ] **Step 4: Measure representative tier durations without CAN**

Plan the same 3-degree J6 motion using `WEB_MOTION_SAFETY` at 100%, 50%, 25%, and 10%. Confirm duration increases monotonically as the tier decreases and the 100% motion no longer inherits the old 2-second floor.

- [ ] **Step 5: Preserve the running service**

Do not restart the live web/CAN service automatically. Report that the Python backend must be restarted after the arm is stopped for the new profile and 10% server default to take effect.
