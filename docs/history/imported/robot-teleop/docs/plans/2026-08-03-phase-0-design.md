# Phase 0 Architecture Design

## Requirements and constraints

Phase 0 must prove that the teleoperation path can be optimized once and reused
across input devices, policies, and robots. It therefore delivers immutable
canonical models, dependency-inverted ports, relative-pose clutching, a safety
state machine, latest-only handoff, a fixed-rate dry-run runtime, mock/replay
adapters, and contract tests. Safety outranks latency: clutch release holds,
E-stop latches, stale inputs progress from hold to brake to backend safe stop,
and hardware remains unavailable by default.

The non-functional target is a 200 Hz servo structure with no unbounded queues,
disk writes, high-frequency logging, network acknowledgements, IK, or model
inference on the control thread. All cross-thread clocks use Ubuntu-local
`monotonic_ns`; a device timestamp is provenance until Phase 1 clock sync makes
it comparable. Python 3.12, frozen dataclasses, explicit registries, short
lifecycle methods, strict typing, and deterministic mocks keep the code easy to
test. Phase 0 must not import or instantiate ARX5 software, touch CAN, or include
an iOS implementation.

## Alternatives considered

The selected approach is a single-process modular monolith with a receiver
thread, servo thread, and metrics thread. Communication between the receiver and
servo is one overwrite-only snapshot. This makes ownership and shutdown visible
while avoiding queue age and serialization overhead.

An asyncio-first runtime was rejected for Phase 0. UDP can fit asyncio, but robot
SDK calls and the fixed-period servo are synchronous; mixing them would obscure
deadline ownership and create accidental event-loop blocking. A multi-process
runtime was also rejected for now. It can isolate future vision or policy models,
but IPC, clock attribution, failure propagation, and deployment complexity add
cost before Python performance has been measured. A two-layer frontend/backend
design was rejected because it couples device semantics to robot execution and
would duplicate clutching, filtering, and watchdog logic when either side is
replaced.

## Components and data flow

Adapters produce `TeleopInput`; the receiver validates sequence/session ordering
and overwrites `LatestValue`. The servo reads one snapshot per deadline, checks
local receipt age through the watchdog, reads canonical `RobotState`, and asks
the core pipeline for a `RobotTarget`. The pipeline owns session continuity,
clutch/recenter edges, tracking state, state-machine transitions, common target
safety, and the sole compliance hook. `RelativePosePolicy` computes an input
delta from captured input and robot references, applies a signed axis map and
position/rotation scale, and composes it with the robot reference. First
activation and recenter therefore produce the current robot pose.

The controller converts a canonical target into a canonical `RobotCommand`.
Phase 0 uses a pass-through Cartesian position controller; Ruckig and IK belong
to later controller/backend composition. The mock backend applies commands to
an in-memory state while enforcing backend-local limits. Neither the core nor
the input adapter knows the robot type. The runtime is the composition root and
is the only layer that imports adapters and core together.

## Failures and verification

Invalid inputs are rejected before publication. Limited or unavailable tracking
cannot generate an active target. A new session or tracking recovery while the
clutch is held requires a release before reactivation. Core safety errors become
FAULT; E-stop becomes a latched ESTOP; runtime exceptions call backend
`safe_stop`. The watchdog uses local receipt time: 50 ms holds, 100 ms requests a
smooth stop command, and 300 ms invokes backend safe stop. Missed servo periods
are recorded and, after falling behind by multiple periods, the deadline is
rebased instead of replayed.

Verification consists of core unit tests, generic input/policy/backend contract
tests, an AST dependency test, and a threaded Mock-to-Mock integration test.
Ruff, mypy strict mode, pytest, and the same commands in GitHub Actions form the
quality gate. Hardware behavior, network packets, Swift vectors, ARX5 SDK calls,
IK, and Ruckig are intentionally outside this phase.

