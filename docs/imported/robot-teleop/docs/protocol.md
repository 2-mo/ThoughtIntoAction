# Binary Protocol Definition

Status: protocol v1.1. Python and Swift codecs implement this document and load
the same checked-in golden vectors.

## Common envelope v1.1

- Byte order: little-endian.
- Quaternion order: `x, y, z, w`.
- Magic bytes on the wire: ASCII `RTLP`.
- Magic as a little-endian `UInt32`: `0x504c5452`.
- The common header is 36 bytes.
- Unknown major versions, packet types, sources, flags, and nonzero reserved
  fields are rejected.
- A v1.1 endpoint accepts the established v1.0 message types and the additive
  v1.1 reset message types. It rejects higher minor versions until their
  compatibility behavior is specified.

| Offset | Type | Field |
|---:|---|---|
| 0 | `UInt32` | magic |
| 4 | `UInt16` | major_version |
| 6 | `UInt16` | minor_version |
| 8 | `UInt16` | packet_size |
| 10 | `UInt16` | packet_type |
| 12 | `UInt16` | source_type |
| 14 | `UInt16` | reserved, zero in v1.0 |
| 16 | `UInt32` | flags |
| 20 | `UInt32` | sequence |
| 24 | `UInt32` | session_id |
| 28 | `UInt64` | client time or echoed client time |

## Message types

| Value | Name | Fixed size | Purpose |
|---:|---|---:|---|
| 1 | `POSE` | 96 bytes | Input pose and controls |
| 2 | `PING` | 36 bytes | Low-rate clock request |
| 3 | `PONG` | 52 bytes | Clock response |
| 4 | `SETTINGS` | 44 bytes | Low-rate canonical mapping scales |
| 5 | `SETTINGS_ACK` | 44 bytes | Applied mapping scales |
| 6 | `ESTOP_RESET_REQUEST` | 36 bytes | Request an explicit local E-stop reset |
| 7 | `ESTOP_RESET_ACK` | 36 bytes | Confirm a completed explicit local reset |

### Pose payload

The Pose header is followed by this 60-byte payload:

| Offset | Type | Field |
|---:|---|---|
| 36 | `Float32` | position_x |
| 40 | `Float32` | position_y |
| 44 | `Float32` | position_z |
| 48 | `Float32` | quaternion_x |
| 52 | `Float32` | quaternion_y |
| 56 | `Float32` | quaternion_z |
| 60 | `Float32` | quaternion_w |
| 64 | `Float32` | linear_velocity_x |
| 68 | `Float32` | linear_velocity_y |
| 72 | `Float32` | linear_velocity_z |
| 76 | `Float32` | angular_velocity_x |
| 80 | `Float32` | angular_velocity_y |
| 84 | `Float32` | angular_velocity_z |
| 88 | `Float32` | gripper |
| 92 | `UInt8` | tracking_state |
| 93 | `UInt8[3]` | reserved, zero in v1.0 |

### Ping and Pong

For Ping, the common header `client_time_ns` field is client send time `t0`.
Pong echoes that header and appends:

| Offset | Type | Field |
|---:|---|---|
| 36 | `UInt64` | Ubuntu receive time `t1` |
| 44 | `UInt64` | Ubuntu send time `t2` |

Pose packets are never acknowledged. Ping is sent at low rate and may retry.

### Settings and SettingsAck

Settings and SettingsAck share an eight-byte payload:

| Offset | Type | Field |
|---:|---|---|
| 36 | `Float32` | position_scale, finite and non-negative |
| 40 | `Float32` | rotation_scale, finite and in `[0, 1]` |

Settings is accepted only after a Pose has established the current session.
Ubuntu applies it to the canonical mapping-settings store and echoes the values
it applied in SettingsAck. A delayed Settings packet from another session is
rejected. Settings changes are latched by `RelativePosePolicy` only on Clutch
activation or recenter, so an active target cannot jump when a slider changes.
These values do not override RobotBackend or target-safety limits.

### E-stop reset request and acknowledgement

Reset messages use only the common header. `sequence`, `session_id`, and
`client_time_ns` form the request identity; the acknowledgement echoes all
three fields. Both require a known input source and zero flags.

An input device sends `ESTOP_RESET_REQUEST` while it remains visibly locked. It
then sends a later valid Pose for the same session with `ESTOP`, `CLUTCH_ACTIVE`,
and `RECENTER` clear. Ubuntu records that request and release proof but retains
its `ESTOP` latch. A local operator must explicitly complete the reset. Only
then can Ubuntu reset its Mock runtime and send `ESTOP_RESET_ACK`; the input UI
unlocks only after validating that exact acknowledgement. Neither reset message
acknowledges Pose packets or grants a remote endpoint authority to clear an
Ubuntu latch.

## Source types

The numeric registry is stable within major version 1:

| Value | Source |
|---:|---|
| 0 | unknown/server |
| 1 | ARKit |
| 2 | VR |
| 3 | SpaceMouse |
| 4 | hand tracking |
| 5 | local teaching device |
| 6 | replay |
| 7 | mock |

Only ARKit is assembled as a live UDP input in Phase 1. Ping and Pong may use
the initiating source; Ubuntu-originated diagnostic traffic may use server zero.

## Flags

| Bit | Name | Meaning |
|---:|---|---|
| 0 | `CONNECTED` | sender session is connected |
| 1 | `CLUTCH_ACTIVE` | hold-to-control input is pressed |
| 2 | `RECENTER` | edge-triggered recenter request |
| 3 | `ESTOP` | latch E-stop request |
| 4 | `TRACKING_VALID` | pose is valid for control |
| 5 | `GRIPPER_VALID` | gripper field is meaningful |
| 6 | `LINEAR_VELOCITY_VALID` | linear velocity is meaningful |
| 7 | `ANGULAR_VELOCITY_VALID` | angular velocity is meaningful |

Pose permits these flags. Ping, Pong, Settings, and SettingsAck require zero
flags. Unknown flag bits are rejected so sender/receiver semantic disagreement
cannot silently activate control. All 14 Pose floats must be finite, including
inactive optional fields; a cleared validity flag converts its field to absent
after validation.

`TRACKING_VALID` must be set exactly when tracking state is limited or normal.
Clutch may be active only while connected and tracking is valid. ARKit supports
the two velocity validity bits and analog gripper. Gripper is in `[0, 1]` when
valid.

## Tracking values

| Value | State |
|---:|---|
| 0 | unavailable |
| 1 | limited |
| 2 | normal |

## Receiver validation order

1. Read the 36-byte header and validate magic and accepted version.
2. Validate known packet type, exact declared size, and datagram length.
3. Validate source, flags, enums, and zero-valued reserved fields.
4. Validate every numeric field is finite.
5. Require raw Pose quaternion norm in `[0.95, 1.05]`, then normalize it.
6. Validate tracking, validity flags, source capabilities, and gripper range.
7. Validate session and sequence ordering.
8. Validate source timestamp monotonicity within the active session.
9. Apply canonical position/rotation jump limits before publication.

Malformed input is rejected and counted without terminating the receiver.

## Session and sequence

`session_id` is a random nonzero `UInt32` generated for each app connection.
Pose `sequence` is a `UInt32` incremented for each new Pose and compared using
serial number arithmetic:

- delta zero is a duplicate;
- delta in `[1, 2^31)` is forward progress, including wrap;
- delta in `[2^31, 2^32)` is old or out of order.

A forward gap is accepted because the newest Pose is valuable, while estimated
loss is counted. A session switch resets sequence comparison and forces core
reference recapture. Recently retired session IDs are held in a bounded set so
delayed datagrams cannot switch control back to an old session.

Ping has an independent request sequence echoed by Pong. Settings and E-stop
reset requests have their own streams and matching acknowledgements. Keeping
these streams separate prevents low-rate control traffic from appearing as Pose
loss. Settings and reset ordering are reset only after an accepted Pose
establishes a new active session.

## Clock synchronization

Client and Ubuntu monotonic clocks are different domains. For client times `t0`
and `t3` and Ubuntu times `t1` and `t2`:

```text
RTT = (t3 - t0) - (t2 - t1)
server_minus_client = ((t1 - t0) + (t2 - t3)) / 2
```

Negative intervals and excessive-RTT samples are rejected. The estimator uses
the median offset from a bounded lowest-RTT sample window. Without a valid clock
estimate, reports show age since Ubuntu receipt and never label raw cross-clock
subtraction as one-way latency.

## Golden vectors

Canonical Pose, Ping, Pong, Settings, SettingsAck, E-stop reset request,
E-stop reset acknowledgement, and JSON manifest fixtures live under
`tests/golden/`. Python and Swift must match every byte and verify the same
field values, sizes, and `xyzw` ordering.

## Network boundary

Phase 1 UDP has no authentication or encryption. Bind it only on a trusted,
isolated control network; do not expose the endpoint to the public internet.
Protocol validation is a safety boundary, not an authentication mechanism.

The UDP byte transport may expose a finite receive batch. A protocol adapter
must process controls in arrival order but publish only the newest valid Pose in
that batch. This preserves Ping/Settings liveness without replaying old motion
targets; see ADR-0008.
