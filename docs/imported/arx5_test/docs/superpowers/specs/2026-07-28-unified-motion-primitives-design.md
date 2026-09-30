# Unified Motion Primitives Design

## Context

The web cube workflow still computes a Cartesian duration with the legacy
`plan_cartesian_trajectory()` path, then passes that duration into the newer
Cartesian waypoint planner. Ordinary XYZ execution no longer does this. The
cube workflow therefore has different timing behavior even though both paths
ultimately command the same robot.

Motion completion also depends on one velocity sample after a fixed settling
delay. A noisy sample such as `0.066 rad/s` can fault an arm whose measured
position is already stationary at the target.

## Goals

- Make joint and Cartesian point-to-point motion controller-owned primitives.
- Let higher-level workflows provide targets and a speed tier, not durations.
- Make preview and execution use the same planning kernels.
- Detect terminal rest from a short feedback window instead of one sample.
- Remove artificial delay from automatic cube-stage transitions.

This change does not migrate real-time servo control, calibration subprocesses,
or gesture teleoperation. Their execution models are intentionally different.

## Controller API

`ArmController` will expose two canonical point-to-point entry points:

- `movej(target_joint_position, *, velocity_scale, allow_large_motion=False)`
  plans and executes one synchronized joint-space S-curve.
- `movep(target_xyz_m, target_orientation_xyzw, *, velocity_scale,
  path_mode="linear", unrestricted=False)` plans and executes one absolute TCP
  target through the shared Cartesian geometry and trajectory pipeline.

Matching `plan_movej` and `plan_movep` methods provide non-commanding previews.
Existing relative and absolute movement methods remain as compatibility
wrappers and delegate to these kernels. They must not maintain independent
duration or derivative policies.

## Data Flow

Higher-level code owns task semantics only:

1. Cube planning produces absolute TCP or gripper targets.
2. The web session snapshots the selected speed tier with the preview.
3. Execution passes each target and the stored velocity scale to `movep`.
4. The controller derives duration, samples, derivative limits, final hold, and
   feedback monitoring from the active motion profile.

No cube action may call a second planner merely to derive a duration. Ordinary
XYZ and cube XYZ execution will therefore share the same code path.

## Terminal Settlement

The existing final zero-velocity hold remains. After that hold, the monitor may
return as soon as feedback is stable for `0.12 s`, up to the existing
`settle_time_s` deadline.

A feedback window is settled when:

- every sample remains inside the configured target position tolerance; and
- the measured position span of every joint stays within `0.15 deg`.

The position-span alternative distinguishes velocity-estimator noise from real
motion even when the reported velocity is noisy or incorrectly zero. Reported
velocity remains diagnostic data rather than a completion gate. If the target
is not stable before the deadline, the existing damping and fault behavior
remains. The error will include target error, observed position drift, and
reported velocity rather than reporting one velocity sample alone.

## Cube Automation

The cube executor will call `movep` with absolute action targets and the speed
stored in the server-side preview. Gripper targets continue through the shared
gripper primitive. The browser's four `1.2 s` automatic preview pauses become
`0.1 s` render pauses; manual preview and confirmation behavior is unchanged.

The default speed remains `10%`. Selecting `100%` uses the configured X5
hardware-based motion profile without a cube-specific duration floor.

## Error Handling

Planning, limit, tracking, and stale-feedback errors continue to abort the
current task. A terminal velocity outlier alone does not fault the controller
when the measured joints are demonstrably stationary at the target. Simulation
state polling remains independent from command success.

## Tests

- Prove cube execution never calls the legacy duration planner.
- Prove cube execution passes the stored speed to the canonical `movep` path.
- Prove a stable-position window accepts a `0.066 rad/s` velocity outlier.
- Prove real position drift still fails at the settling deadline.
- Prove ordinary XYZ and cube XYZ use the same controller primitive.
- Update the frontend contract for the `0.1 s` automatic render pause.
- Run the complete Python suite and JavaScript syntax checks.
