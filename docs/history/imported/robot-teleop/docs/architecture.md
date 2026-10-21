# Architecture

## Scope

Phase 1 provides two hardware-free Ubuntu paths: Mock input to Mock robot and
ARKit-format UDP to Mock robot. Phase 2 adds a Swift/SwiftUI client with ARKit
and simulated pose providers, shared golden codecs, latest-only UDP, clock sync,
and session controls. Phase 3 adds a v1.1 two-sided E-stop reset contract and a
deterministic ARKit-format UDP-to-Mock validation scenario. Its simulator path,
direct physical iPhone UDP, primary controls, background release, and
process-reset recovery are verified; connected-iPhone Mock reset and visual
coordinate review remain operator checks. Phase 4 adds an offline X5 URDF,
Pinocchio FK/Jacobian, and bounded seed-continuous IK preflight outside the live
control path. The ARX5 backend, CAN, online Ruckig, and compliance control
remain later phases.

## Five layers

```text
Input device adapter
        |
Transport and decoder
        |
Canonical teleoperation core
        |
Control policy and controller
        |
Robot backend
```

The canonical path is:

```text
device bytes/state
  -> TeleopInput
  -> latest-only snapshot
  -> Session + state machine + mapping + common safety
  -> RobotTarget
  -> Controller
  -> RobotCommand
  -> RobotBackend
  -> RobotState
```

`RobotTarget` expresses policy intent. `RobotCommand` expresses what a backend
can execute after controller-level shaping. This boundary leaves room for IK and
online Ruckig without putting either in an input adapter or canonical policy.

## Dependency rule

```text
cli / registry / runtime / adapters / policies / controllers
                              |
                              v
                            core
```

`robot_teleop.core` uses the Python standard library only. It cannot import
socket APIs, filesystems, ARKit/Swift, ARX5, UR RTDE, ROS/MoveIt, FastAPI, robot
URDF paths, or any concrete adapter. Port protocols live in `core.ports`; each
adapter `base.py` re-exports the corresponding port for discoverability.

The composition root is `registry.py` plus `runtime/application.py`. Registry
entries are explicit dictionaries; there is no dynamic discovery or dependency
injection framework.

## Ownership

| Concern | Single owner |
|---|---|
| Canonical geometry and `xyzw` convention | `core/frames.py`, `core/models.py` |
| Clutch, recenter, session continuity | `core/session.py` |
| State and latched E-stop | `core/state_machine.py` |
| Reset readiness and explicit local completion | `runtime/estop_reset.py`, `runtime/application.py` |
| Axis and scale mapping | `core/mapping.py`, selected policy |
| Common input/target limits | `core/safety.py` |
| Compliance hook | one `CompliancePolicy` in the pipeline |
| Network/input freshness | `runtime/watchdog.py` |
| Wire layout and semantic validation | `protocol/packet.py` |
| Source sequence/session ordering | `protocol/sequence.py` |
| Clock RTT/offset estimation | `protocol/clock_sync.py` |
| Bounded socket drain and protocol-aware Pose coalescing | `transports/udp.py`, `inputs/arkit.py` |
| ARKit capability boundary | `inputs/arkit.py` |
| Latest-only handoff | `runtime/latest_value.py` |
| Deadlines and command cadence | `runtime/servo_loop.py` |
| Runtime/protocol distributions | `runtime/metrics.py` |
| Offline model/FK/IK/Jacobian preflight | `kinematics/` and Phase 4 tool |
| Generic online joint Ruckig state | `controllers/ruckig_servo.py` |
| Static ARX5 package metadata inspection | `tools/arx5_sdk_inspect.py` |
| Robot-specific hard limits and stop | each `RobotBackend` |
| Component construction | `registry.py` |
| iOS frame ownership and tracking conversion | `Tracking/*PoseProvider.swift` |
| iOS packet/session scheduling | `App/TeleopCoordinator.swift` |
| iOS bounded Pose transmission | `Transport/UDPClient.swift` |
| iOS operational state and controls | `App/AppModel.swift`, `UI/` |

## Threads and timing

The application runs three joinable threads:

1. Receiver polls Mock input or blocks on UDP, validates canonical ordering,
   and overwrites a single latest snapshot.
2. Servo runs at the configured rate (200 Hz by default), reads at most one
   snapshot, generates at most one current command, and records timing in
   memory.
3. Metrics periodically computes summaries from bounded in-memory samples.

The receiver never calls a robot, controller, IK, or filesystem. The servo never
writes files, logs per cycle, acknowledges network packets, or catches up missed
historical periods. If it is behind by more than two periods, it records the miss
and rebases the next deadline on current monotonic time.

`TeleopInput.timestamp_ns` remains source-clock provenance. Freshness is driven
by `LatestValue.stored_at_ns`, captured with host `time.monotonic_ns()`, so raw
client/server clock subtraction is never called one-way latency. Low-rate
Ping/Pong supplies four timestamps for client-side RTT and offset estimation.

On iOS, the ARKit delegate or simulator provider creates source samples. One
serial user-interactive coordinator queue owns session, sequence, encoding, and
control scheduling. Network.framework owns socket callbacks. The sender keeps
at most one Pose in flight and one replaceable pending Pose; UI state returns to
the main actor. Haptics and rendering never gate packet transmission.

UDP reception performs one blocking read followed by a bounded nonblocking
drain. Generic transport callers can request only the newest byte datagram. The
ARKit adapter consumes the bounded batch, processes every control message in
arrival order, and publishes only the newest valid Pose. Transport still owns
only bytes and peer addresses; `ArkitPacketDecoder` owns source and capability
semantics. Pose is never acknowledged. Protocol traffic is accepted only on a
trusted network; v1 has validation but no authentication or encryption.

Pose is intentionally best-effort. Low-rate Ping and Settings exchanges are
matched to session, sequence, and timestamp; Ping times out and Settings retries
until acknowledged. This tolerates loss or bounded receive-side supersession
without creating a reliable queue of historical Pose targets.

## State and failure flow

```text
DISCONNECTED -> READY -> ACTIVE -> HOLD
                    |        |
                    v        v
                 DEGRADED  DEGRADED

Any non-latched state -> ESTOP (explicit reset required)
Any runtime/safety fault -> FAULT (explicit reset required)
```

For the v1.1 E-stop reset path, the phone sends a session-scoped reset request
and then a newer released Pose with normal tracking. `EStopResetGate` marks that
proof ready but cannot reset the latched pipeline. A local
`TeleopApplication.reset_estop()` call re-arms only the injected Mock backend,
resets the Servo latches, and sends the matching acknowledgement. The iOS UI
stays locked until that exact acknowledgement arrives. No production backend is
registered for this recovery path.

Clutch activation captures both current input and current robot references.
Release generates HOLD immediately. A changed session or tracking recovery while
the clutch remains pressed requires release before another activation. A stale
network interval and a runtime reconnect follow the same rule. Recenter captures
new references from current states, so its first target is unchanged.

The Watchdog arms on the first accepted normal-tracking input with an active
clutch (or an explicit E-stop). Before active control begins, the Servo remains
stationary while release-only startup baselines are received, so an operator
may start the host console before connecting the input device without turning a
transient ARKit/network initialization gap into a latched fault. Once armed,
input-age timeout handling remains active across clutch release and tracking
recovery.

For a Cartesian position target created from a new input, the common target-rate
limit uses the interval between distinct host-received input snapshots. The
controller still runs at the fixed Servo rate, but a 60 Hz input source must not
be measured as though every target change occurred at 200 Hz. A burst received
by the host still has a short interval and remains subject to the stricter
limit.

Once armed, Watchdog stages use local input receipt age:

| Age | Runtime action |
|---:|---|
| `<= 50 ms` | normal pipeline |
| `> 50 ms` | hold current robot state |
| `> 100 ms` | issue controller/backend STOP intent |
| `> 300 ms` | invoke `RobotBackend.safe_stop()` and end servo |

Robot feedback timestamps are monitored independently. A timestamp that stops
advancing beyond the configured threshold is a fault and invokes backend safe
stop.

## Extension invariants

- A new input implements `TeleopInputSource`; core, controller, and robot code do
  not change.
- A new robot implements `RobotBackend`; input, session, mapping, filtering, and
  watchdog code do not change.
- A new policy implements `ControlPolicy`; input and SDK adapters do not change.
- Robot-specific quaternion ordering is converted once inside its backend.
- Frontend scale requests can only reduce trusted limits; they cannot expand a
  backend safety envelope.

## Non-functional requirements

- Hard-safety-bounded, responsiveness-optimized, and dry-run-first lifecycle.
- 200 Hz initial servo baseline, not a ceiling or permanent control rate.
- Constant memory in the live command path.
- Graceful stop for all worker threads and resources.
- Strict typing and deterministic tests without optional robot dependencies.
- No high-frequency JSON, disk I/O, GUI, video, or model inference in the path.

The optional `kinematics` extra is a tool-only dependency. It may load a URDF
and allocate Pinocchio model data for offline review, but it is not imported by
`core`, Receiver, Servo, or the default test environment. No kinematics model
can initialize a robot SDK, CAN interface, or hardware backend.

The optional `arx5` extra contains the vendor wheel, Pinocchio, and upstream
Ruckig. Importing the Registry or constructing `Arx5Backend` does not import
the wheel. SDK construction is possible only after both the CLI hardware gate
and an explicit interface name reach `Arx5Backend.connect()`. The current
backend rejects active Cartesian commands and enters its stop boundary until
robot-specific IK, limits, gripper calibration, and stop semantics are verified.
It also requires a measured-joint FK provider instead of treating SDK EEF state
as confirmed TCP feedback. `RuckigJointServo` is generic and holds no SDK or
transport reference.

Common Watchdog, filtering, recovery, and motion-shaping values are configurable
policies. They may be tuned or replaced from measured latency, jitter, target
continuity, and stopping behavior, provided verified backend hard limits and
explicit hardware gates remain intact. See ADR-0005.

See the accepted decisions in [`docs/adr`](adr/README.md).
