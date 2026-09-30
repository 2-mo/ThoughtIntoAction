# Unified Cartesian Trajectory Speed Design

## Status

Approved direction: one motion-safety profile. Speed tiers change only the
time required to execute a fixed geometric and IK path.

## Goals

- Generate one smooth Cartesian geometry and one continuous IK solution for a
  target, independent of the selected speed percentage.
- Make 25%, 50%, and 100% executions differ only by their time scale.
- Validate the exact timestamps and command velocities that will be submitted.
- Remove the current failure mode where low-speed scaling repeatedly tightens
  derivative limits and rejects a normal path at the 120-second cap.
- Keep software joint limits, FK/IK residual checks, TCP obstacle checks,
  feedback velocity/torque checks, and emergency damping.

## Non-Goals

- No obstacle avoidance or global motion optimization.
- No automatic enabling of the uncalibrated capsule self-collision model.
- No hardware-specific maximum-speed profile in the web control protocol.

## Design

### 1. Single motion profile

The web service uses one `SafetyConfig` for all speed percentages. The
`safe/hardware_max` request field and UI toggle are removed. The configured
limits are the hard envelope for every command; the speed percentage is a
normalized time-scale input in `(0, 1]`.

### 2. Speed-independent geometry and IK

Cartesian geometry is sampled by spatial distance and orientation distance, not
by a speed-dependent Ruckig sample count. The resulting ordered poses are
solved once with the current joint state as the seed. Each waypoint is checked
for joint limits, FK position/orientation residuals, singularity, TCP
obstacles, and the opt-in link collision check.

The preview stores the canonical waypoint path. Execution verifies that the
measured pose and joints have not drifted beyond the existing preview
tolerances, then uses the stored path rather than resolving a new IK path.

### 3. Canonical time parameterization and interpolation

At 100%, a scalar path-progress trajectory is generated under the single
profile's Cartesian velocity, acceleration, and jerk caps. Joint-space
derivatives are checked against the same canonical timestamps. Any additional
time stretch needed for the hard envelope is applied once.

For a requested speed scale `s`, every relative timestamp is multiplied by
`1 / s` and every commanded waypoint velocity is multiplied by `s`. The
resulting acceleration and jerk decrease naturally by `s^2` and `s^3`; no
second IK solve or independent derivative-limit scaling is performed.

The canonical path is treated as a continuous joint trajectory, not as a
list of hold points. After time scaling, it is resampled at the fixed SDK
control period (with a bounded maximum period when a very short path has only
a few anchors). Position, velocity, and acceleration are reconstructed with a
quintic Hermite interpolant between canonical samples, so slow execution adds
command samples without introducing position steps, velocity or acceleration
jumps, or dwell-like pauses.
Endpoint velocities remain zero for Cartesian command boundaries. The
resampled position, velocity, acceleration, and jerk arrays are the exact
arrays passed to the validator and SDK.

### 4. Duration policy

The 120-second fixed guard is replaced by a single explicit operation limit of
300 seconds. Previews report the calculated duration before execution. A path
that exceeds the operation limit is rejected as an unusable speed selection,
not reported as an acceleration failure. The lowest speed tiers remain
available for short moves; long cube operations should be rejected at preview
time with the measured duration.

### 5. Error and safety handling

Planning and submission call the same derivative validator with the same
timestamp array. Cartesian paths use a zero command velocity boundary after
the existing stationary check; Ruckig joint curves retain their measured
initial velocity. Terminal web position tolerance is 2 degrees and terminal
velocity tolerance remains 0.04 rad/s. A safety violation still aborts and
enters damping.

## Data Flow

```text
measured state
    -> fixed spatial/orientation samples
    -> continuous IK + safety checks
    -> canonical 100% timestamps and velocities
    -> speed-scale timestamps/velocities
    -> exact-sample validation
    -> SDK trajectory submission
```

The preview and execution paths share the canonical sample representation.
Only the timestamp and velocity arrays are derived from the selected speed
scale.

## Verification

- Unit test that all speed scales produce identical joint waypoint positions.
- Unit test that timestamps scale as `1/s` and velocities as `s`.
- Unit test that slow resampling increases sample density while preserving
  continuous position and velocity at every interpolation boundary.
- Unit test that acceleration and jerk remain within the single profile after
  scaling.
- Regression test for nonzero measured feedback velocity at a Cartesian
  command boundary.
- Cube-path integration test at 25%, 50%, and 100% with the same IK path.
- Full `./arx5 test` regression and a no-motion SDK preview for the reported
  cube coordinate.
