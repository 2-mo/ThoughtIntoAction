# Safety

## Safety claim boundary

Phase 1 is dry-run only and makes no real-time or machinery-safety certification
claim. Software E-stop is not a physical emergency stop. No command in this
repository should be interpreted as authorization to move hardware unattended.

## Common layer

The canonical/runtime layer owns input ordering, tracking quality, session and
clutch continuity, recenter, input jumps, target steps, workspace policy,
network freshness, E-stop latching, and state transitions. It operates only on
canonical models and cannot weaken backend limits.

Common thresholds are policy limits, not immutable hardware facts. They should
be tuned for low latency and continuous control using measured network jitter,
robot stopping behavior, and target continuity. They may approach, but never
exceed, verified backend hard limits.

## Robot layer

Each backend owns SDK lifecycle, joint position/velocity/acceleration/jerk and
torque limits, CAN/SDK health, feedback freshness, singular regions,
robot-specific stop behavior, and final command validation. A frontend scale is
advisory and can only reduce a trusted backend envelope.

The ARX5 implementation has a separately injectable safety-envelope boundary
for these checks. Static package and local-reference values are candidates, not
physical-arm limits, so the default backend does not inject them and its active
command path remains disabled. On the active-boundary configuration, malformed
or stale feedback latches the backend and attempts the documented SDK damping
primitive once; a damping exception remains an explicit fault rather than a
claimed stop. The state-only readiness tool disables framework damping on
feedback failure so its no-command contract stays intact.

## Network boundary

Protocol v1 rejects malformed lengths, versions, types, sources, flags,
reserved fields, NaN/Inf, bad quaternion norms, capability mismatches,
duplicates, out-of-order packets, retired sessions, non-monotonic timestamps,
and implausible canonical jumps. This validation does not authenticate a sender.
UDP must bind only on a trusted, isolated control network and must not be
published to the internet.

The socket and runtime both discard superseded targets. Ping/Pong is low-rate;
Pose has no ACK or retransmission. Host Watchdog age uses Ubuntu receipt time,
not raw subtraction from the iPhone clock.

## Watchdogs

The current defaults are configurable starting points:

- receipt age above 50 ms: stop extrapolation and hold;
- receipt age above 100 ms: request limited braking to zero velocity;
- receipt age above 300 ms: call backend `safe_stop`;
- feedback timestamp stall: enter FAULT and call backend `safe_stop`.

Host receipt time, not an unsynchronized device timestamp, drives these stages.
The Watchdog arms after the first accepted `tracking=normal` input with
`clutch_active=true` (or an explicit E-stop). Before active control begins,
release-only startup baselines remain stationary without turning a transient
ARKit/network initialization gap into a latched fault. Once armed, the same
staged timeouts apply to every loss of the command stream and remain armed
across clutch release or tracking recovery.

For new Cartesian position inputs, target velocity is evaluated against the
interval between their Ubuntu receipt times, rather than the fixed Servo period.
This preserves the configured limit for the source trajectory without treating
a normal 60 Hz input update as a 200 Hz jump. Target step limits remain active,
and Cartesian position targets that exceed the step/rate envelope are advanced
only by the permitted amount toward the newest input. Repeated Servo cycles do
not advance that target again without a new input snapshot, so this is a
latest-only slew limit rather than a queued trajectory. Packets that arrive in
a host-side burst are still checked against their short observed interval.
The ordering of hold, brake, and final safe stop remains meaningful, but the
values and recovery policy may change after failure-path testing. In particular,
brief degradation should not force unnecessary stop-start interaction when a
continuous, bounded recovery is demonstrated to be equally safe.

## Responsiveness boundary

The safety layer should reject genuinely stale, discontinuous, or out-of-envelope
motion; it should not become an untuned source of lag. Filters are optional,
deadzones stay as small as measurements allow, and normal in-envelope motion
should not be clamped below the robot's validated smooth-control capability.

Non-negotiable boundaries are limited to explicit hardware enablement, latched
E-stop, truthful physical E-stop documentation, and RobotBackend hard limits.
Control frequency, Watchdog values, filter parameters, recovery interaction, and
common limits below that envelope are reviewable tuning decisions.

## State rules

- E-stop and FAULT are latched; recovery requires an explicit reset path.
- Tracking limited enters DEGRADED and cannot produce active motion.
- Tracking unavailable enters HOLD.
- Clutch release enters HOLD immediately.
- Session change and tracking recovery while pressed require release/re-press.
- Network-stale recovery and runtime restart require release/re-press.
- Any uncaught servo, controller, or backend exception attempts backend safe
  stop before the worker exits.

## Hardware prerequisites for later phases

- Operator physically beside the arm.
- Stable arm mounting and cleared workspace.
- Tested physical emergency stop within reach.
- Robot model/interface identity confirmed.
- Safe stop tested before motion.
- First motion limited to one axis at very low speed.
- No Codex or CI process starts hardware motion automatically.
