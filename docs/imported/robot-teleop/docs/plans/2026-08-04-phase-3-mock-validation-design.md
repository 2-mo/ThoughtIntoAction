# Phase 3 Mock Validation Design

Date: 2026-08-04

Status: approved for implementation on `feat/phase-3-mock-validation`.

## Scope

Phase 3 validates the direct iPhone/UDP/canonical-runtime path against
`MockRobotBackend`. It does not register a real robot, initialize CAN, load an
URDF, perform IK, or create a Ghost visualizer. Those remain Phase 4 and Phase
5 work.

The automated portion must be runnable on one development machine. The later
operator run sends real iPhone ARKit traffic directly to the Ubuntu Mock
receiver and writes a finite JSON report. A physical direction review remains
an operator gate; it is not inferred from packet arithmetic alone.

## Reset handshake

The E-stop reset path is intentionally separate from the high-rate Pose stream.
It adds two header-only protocol messages in v1.1:

```text
iPhone                            Ubuntu Mock runtime
------                            -------------------
E-stop Pose --------------------> pipeline/backend latch ESTOP

ResetRequest -------------------> record pending request only
released Pose ------------------> validate same session, normal tracking,
                                  clutch released, E-stop clear

                                  local operator explicitly calls reset_estop()
ResetAck <----------------------- reset Mock backend/pipeline, then acknowledge
unlock UI <---------------------- only after matching ResetAck
```

The phone remains visibly locked from the reset request until the matching
acknowledgement. It switches only its on-wire E-stop bit off while awaiting that
acknowledgement, sends a Clutch-released safety Pose, and cannot reactivate
control. Ubuntu does not clear its E-stop on a request, acknowledgement, or
ordinary Pose. Its local `reset_estop()` requires all of these conditions:

1. The canonical state is `ESTOP`, not `FAULT`.
2. The request belongs to the currently established ARKit Pose session.
3. A later valid Pose from that session has normal tracking, E-stop clear,
   Clutch released, and no Recenter edge.
4. The caller explicitly invokes the reset method.

For Phase 3 only, the composition root injects Mock's explicit stationary
re-arm callback. No real backend implements this callback, so this mechanism
cannot accidentally claim a hardware recovery procedure.

## Boundaries and ownership

```text
Swift ControlStateStore
  -> TeleopCoordinator -> ResetRequest / released Pose
  -> UDP transport -> ArkitInputSource
  -> EStopResetGate -> TeleopApplication.reset_estop() [local action]
  -> Mock re-arm + ServoLoop reset -> ResetAck
```

`PacketCodec` owns the fixed wire layout. `ArkitInputSource` validates active
session and reset-request sequence ordering, but has no state-machine or robot
knowledge. The thread-safe `EStopResetGate` is runtime-only: it correlates an
already validated request with an already validated safe release. The canonical
pipeline continues to own E-stop latching. `TeleopApplication` is the only
component that can invoke the explicit reset transition.

The generic `core/` package remains unaware of UDP, ARKit, packet types, and
the reset handshake. The Mock re-arm callback is passed by the registry rather
than imported by runtime code.

## Validation harness

The Phase 3 harness runs an actual `UdpTransport`, `ArkitInputSource`,
`InputReceiver`, `ServoLoop`, and `MockRobotBackend`. Deterministic datagrams
exercise mapping, Clutch, Recenter, tracking, session changes, packet errors,
watchdog stages, E-stop reset, and bounded latest-only delivery. Its report
contains current metrics plus explicitly measured command-transition latency.

`tools/phase3_mock_session.py` is a separate operator aid. It never enables
hardware and accepts only a local terminal command to request the explicit Mock
reset. It records metrics to JSON after a bounded duration; it does not print
from the Servo or receiver threads.

## Failure behavior

- Unknown newer minor protocol versions, malformed reset packets, old sessions,
  and out-of-order reset requests are rejected and counted.
- A reset request during `FAULT`, without a safe release, or after a session
  change cannot unlock either side.
- A lost ResetAck leaves the phone locked. Repeating the same request lets the
  already completed Ubuntu reset re-send the same acknowledgement without
  restarting motion.
- A network interruption while the phone awaits acknowledgement restores the
  phone's on-wire E-stop request before it reconnects.
- The validation harness has no unbounded packet, command, or metric queue.

## Alternatives considered

### Phone-only clear

Rejected because it can make the UI look active while Ubuntu remains latched.

### Clear Ubuntu as soon as a ResetRequest arrives

Rejected because a network packet is not local operator authorization and the
phone may still be transmitting E-stop Poses.

### Add a separate reset service or HTTP API

Rejected for Phase 3 because it adds a network authority surface and breaks the
small direct-runtime model. A local application method plus an operator tool is
sufficient for Mock validation.

### Move Ghost visualization into this phase

Rejected because the Phase 4 boundary is deliberate. Phase 3 establishes
protocol and continuity evidence first; visualization must not change the
receiver or Servo cadence.

## Verification matrix

| Scenario | Automated now | Operator later |
| --- | --- | --- |
| UDP protocol and reset byte compatibility | yes | n/a |
| Mock mapping and continuity | yes | review with real input |
| Tracking/session/watchdog/error rejection | yes | iPhone lifecycle run |
| E-stop local reset contract | yes | local Ubuntu command + phone UI |
| Ten-minute bounded run | yes, configurable duration | yes, iPhone stream |
| Axis/handedness physical approval | no | required before Phase 4 exit |

