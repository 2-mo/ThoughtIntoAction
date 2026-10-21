# Phase 5 Foundation Report

Date: 2026-08-05

## Delivered

- Added an optional `arx5` dependency extra:
  `arx5-interface==0.1.3` (Linux x86_64 marker), `pin==4.1.0`, and
  `ruckig==0.19.4`.
- Added `Arx5Backend` with an import-isolated lifecycle, explicit hardware and
  interface gates, feedback timestamp validation, fail-safe damping boundary,
  and an injected measured-joint FK requirement for runtime TCP state.
- Converted the evidenced SDK EEF state layout (`XYZ + fixed-axis RPY`) to the
  canonical normalized `xyzw` quaternion in a separate backend-boundary helper.
  The runtime does not treat that EEF field as measured TCP feedback.
- Added `RuckigJointServo`, a robot-independent latest-target online wrapper
  that carries position, velocity, acceleration, and jerk state forward every
  fixed-period step. It keeps no historical target queue.
- Registered `arx5` explicitly. A profile without `--enable-hardware` is
  rejected, and an enabled profile without `--robot-interface` is rejected
  before SDK loading.
- Added fake SDK and optional Ruckig tests plus an isolated CI job. No SDK
  controller is constructed by tests or CI.
- Added `arx5-sdk-inspect`, a controller-free static metadata tool for the
  packaged X5 model/configuration. It has no interface argument and cannot open
  CAN.
- Added `phase5-sdk-model-check`, which rejects a static SDK/offline-model
  mismatch in frame names, joint count, position limits, or velocity limits.
- Added `CartesianRuckigServo`, a controller-independent, hardware-free
  composition of seeded Cartesian IK and latest-target Ruckig. It rejects an
  unreachable or discontinuous branch before replacing the prior target and
  supports an explicit smooth zero-velocity brake request.
- Added `phase5-motion-preflight`, which runs the deterministic mapped
  Clutch/Recenter/rotation path through the attributed X5 model at 200 Hz.
  It imports no SDK and has no CAN interface argument.
- Added `arx5-state-readiness`, a state-only readiness tool. Its default path
  is a dry run with no SDK import, controller construction, CAN access, or
  command write. Hardware access requires all explicit state-only flags and
  records SDK timestamp, joint feedback, gripper raw readout, and offline-FK
  readiness.
- Added `phase5-hold-current`, an explicitly gated, short current-joint hold
  validation. It requires hardware enablement, operator presence, an interface,
  and a separate command-write flag, records drift/timestamp evidence, and
  requests damping on exit. It is not wired to teleoperation or the Servo loop.
- Added `phase5-safety-contract`, a controller-free report that combines static
  package position/velocity/torque metadata with conservative `arx5_test`
  `fine` profile and workspace candidates. It always reports hardware disabled,
  marks the candidate envelope unverified, and lists the evidence still needed
  before any command path can consume it.
- Added an injectable `Arx5SafetyEnvelope` at the backend boundary. Fake and
  pure-envelope coverage verifies position, velocity, acceleration, jerk,
  torque, workspace, Cartesian velocity, and singularity checks; the backend
  applies the relevant feedback/target checks while trajectory commands remain
  disabled. The default backend has no injected envelope and no active command
  path.
- A malformed or stale active-boundary feedback sample now latches the backend,
  blocks subsequent commands, and makes one best-effort damping call. A damping
  exception remains observable as a failure; it is never treated as a confirmed
  stop. State-only readiness deliberately disables the framework damping path.

## Evidence Reviewed

- `arx5-interface==0.1.3` package metadata and upstream `real-stanford/arx5-sdk`
  API declarations.
- `/home/r2/Desktop/arx5_test` at the SHA recorded in
  [`references.md`](references.md), including its state conversion, lifecycle,
  online Ruckig, and stale-feedback code.

No source was copied from `arx5_test`; the only copied asset remains the
MIT-attributed X5 URDF documented in `assets/robots/arx5/README.md`.

## Explicitly Not Enabled

- Default commands, CI-equivalent tests, static inspection, motion preflight,
  and the safety-contract report do not open a CAN interface or construct an
  `Arx5JointController`.
- Active SDK Cartesian/Ruckig command wiring, robot-specific hard limits,
  gripper calibration, and measured feedback semantics are not accepted yet.
- `set_to_damping` is only accepted as a gated SDK damping/safe-stop primitive;
  it is not an active motion primitive and does not authorize Cartesian, joint,
  or gripper commands.
- The readiness tool deliberately keeps state-only reads separate from
  damping/safe-stop validation. It configures the backend not to call
  `set_to_damping()` on readiness disconnect, readiness connect-error, or
  readiness feedback-fault paths.
- Operator state-only evidence showed that the vendor SDK lifecycle itself opens
  SocketCAN, starts background send/receive, clears controller state, and logs
  `Set to damping before exit`. That behavior is outside the framework's target
  command path but means ARX5 state readiness is not a purely passive bus read.

## Ubuntu Static Package Evidence

The Ubuntu `r2` host ran the isolated package check against
`arx5-interface==0.1.3`; it reported `hardware_enabled=false`,
`controller_constructed=false`, and `can_interface_opened=false`.

| Static field | Observed package value |
|---|---:|
| Joint count | 6 |
| Base / EEF frames | `base_link` / `eef_link` |
| Controller period | `0.002 s` |
| Controller default I/O mode | `background_send_recv=true` |
| Controller default interpolation | `linear` |
| Gripper width | `0.088 m` |
| End-effector velocity defaults | `[0.6, 0.6, 0.6, 1.8, 1.8, 1.8]` |

The static position and velocity limits matched `configs/robots/arx5.yaml`.
These values remain package defaults, not verified limits or behavior of the
specific physical arm.

## Candidate Safety Contract

`phase5-safety-contract` is an evidence-reconciliation tool, not a motion or
readiness command. On Ubuntu x86_64 it reads the same static SDK metadata as
`phase5-sdk-model-check`, validates the vectors' dimensions, and produces an
unverified `Arx5SafetyEnvelope` candidate. The candidate intersects the package
position/velocity/torque fields with the local reference's conservative `fine`
profile values: `1.5 rad/s`, `0.30 rad/s^2`, `1.0 rad/s^3`, `0.03 m/s`, and
`0.35 rad/s`. It also records the reference workspace candidate
`X 0.15..0.65 m`, `Y -0.35..0.35 m`, and `Z 0.00..0.55 m`.

The lower Z value is chosen as the more conservative documented reference, not
as a resolved physical-arm fact. The report deliberately remains blocked on
arm identity/firmware/units/command-mode confirmation, gripper calibration,
workspace lower-Z confirmation, and singularity thresholds. It accepts no CAN
interface argument and never constructs a controller.

## Local arx5_test Evidence

The Ubuntu checkout at `/home/r2/Desktop/arx5_test` was re-read at
`69a144f1cd7b6157a536428096b1f52e25d9200e`. It is a complete application
reference, but it is not imported as this repository's architecture.

- Likely deployment defaults: X5, `can0`, gripper width `0.088 m`, and
  gripper open readout `-3.24846` from
  `calibrations/gripper/calibration.json`.
- Teleoperation workspace reference: `X 0.15..0.65 m`,
  `Y -0.35..0.35 m`, and config `Z 0.0..0.55 m`. The reference safety docs
  also describe `Z 0.00..0.55 m`, so the lower Z bound must be reconciled
  before any hardware command path.
- Profile references: `fine` uses translation/rotation gains `0.5/0.6`,
  TCP linear/angular limits `0.03 m/s` and `0.35 rad/s`, and joint
  `1.5 rad/s`, `0.30 rad/s^2`, `1.0 rad/s^3`. `fast` uses
  `1.0/1.0`, `0.10 m/s`, `0.80 rad/s`, and joint `3.5 rad/s`,
  `0.70 rad/s^2`, `3.0 rad/s^3`.
- Stop/lifecycle reference: the application connects through the SDK, validates
  feedback, enters damping, can stabilize by holding the first measured pose,
  uses latest target generation, brakes on teleoperation watchdog timeout, and
  falls back to `set_to_damping()` on interrupted startup or emergency stop.

These are adopted as candidates for state-only and low-speed validation. They
do not by themselves authorize live Cartesian command wiring.

## State-Only Hardware Reads

The initial two-sample operator run constructed the SDK controller on `can0`
and returned joint/gripper feedback, but both samples reported
`sdk_timestamp_s=0.150764`. It was retained as a failed timestamp-progress
probe, not as readiness evidence.

The operator then supplied a five-sample rerun. It returned
`readiness_passed=true`, `ready_for_hardware_state_read=true`,
`controller_constructed=true`, `can_interface_requested=true`,
`command_writes_enabled=false`, and `read_error=null`. The SDK timestamp
sequence was:

```text
0.150835, 0.250345, 0.349863, 0.451669, 0.550924
```

`timestamp_progress=true`, `host_sample_progress=true`, and all five state
reads completed without a framework Servo target, joint target, or gripper
command. The observed absolute maxima over the samples were:

- Joint velocity rad/s: `[0.01319, 0.05714, 0.47033, 0.27473, 0.01099, 0.03297]`
- Joint torque Nm: `[0.41641, 1.13436, 5.90154, 1.30275, 0.00932, 0.00932]`
- Gripper raw position: `-0.0000834` (stable across all samples)

The vendor log confirms that SDK construction created and bound the SocketCAN
socket, cleared motor-controller state, started background send/receive, and
entered damping on exit. This is outside the framework command path, but the
state-only check is not a passive bus read. The run also emitted a KDL warning
that root-link inertia is unsupported; no dynamics or torque claim is made from
this state-only result, and that model warning remains to be reconciled.

The five-sample run accepts the state-only timestamp and feedback-transport
gate. It does not by itself verify physical hard limits, firmware or units, or
gripper calibration. The observed gripper readout remains materially different
from the local reference's `-3.24846`, so gripper normalization and commands
stay disabled.

## Safe-Stop Hardware Damping

The gated safe-stop validation was run on the target arm over `can0` after the
operator confirmed presence, physical E-stop reachability, and a cleared
workspace. The tool constructed the SDK controller, sent exactly one
`set_to_damping()` request, and did not send any framework Servo, Cartesian,
joint-position, or gripper command.

The accepted run returned `safe_stop_passed=true`,
`stop_command_sent=true`, `stop_command_error=null`, advancing SDK and host
timestamps, and no blocking reasons. It collected one pre-stop sample and three
post-stop samples; the maximum post-stop joint velocity was
`0.12088 rad/s`, below the configured `1.0 rad/s` gate.

## Offline Motion Preflight

The deterministic nine-target mapped Clutch/Recenter/rotation path completed
through Pinocchio FK, seeded IK, and latest-target Ruckig at 200 Hz. Every
target was accepted and finished without a historical target queue. The small
test path reached `0.234 rad/s` maximum observed joint velocity under an
explicit offline benchmark profile of `3.5 rad/s`, `0.70 rad/s^2`, and
`3.0 rad/s^3` limits. Those benchmark values were read from the reviewed local
reference's `fast` profile; they are not hardware limits and cannot enable an
ARX5 command path.

## Verification

On the Mac development environment:

```text
make check: 245 passed
make phase5-check: 49 passed
make phase5-motion-preflight: nine deterministic segments accepted and finished
```

The default test environment remains usable without the optional ARX5/Pinocchio
packages; the optional package tests are isolated and skipped when those extras
are unavailable.

## Next Gate

Before wiring live Cartesian commands, reconcile the five-sample state-only
evidence with the exact arm variant, firmware, CAN interface, SDK timestamp
units, gripper calibration, workspace lower Z bound, verified joint and
derivative limits, and singularity thresholds. The fake-backed state-only and
safety-envelope boundary tests are in place; physical safe-stop evidence is
accepted, while physical limit evidence remains required.

Local dry-run command (does not require the vendor SDK):

```bash
uv run --extra arx5 python tools/arx5_state_readiness.py
```

Controller-free static package evidence command (Ubuntu x86_64 with the
optional `arx5` extra installed):

```bash
uv run --extra arx5 python tools/phase5_safety_contract.py
```

Operator state-only command used for the accepted five-sample run, after
physical E-stop and cleared workspace were confirmed:

```bash
uv run --extra arx5 python tools/arx5_state_readiness.py \
  --enable-hardware \
  --state-only \
  --robot-interface can0 \
  --confirm-operator-present \
  --samples 5 \
  --sample-interval-ms 100
```
