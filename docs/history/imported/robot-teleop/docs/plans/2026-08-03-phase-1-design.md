# Phase 1 Protocol and Ubuntu Network Design

## Scope

Phase 1 builds the complete Python-side network path:

```text
UDP datagram
  -> fixed binary decoder
  -> ARKit input adapter
  -> canonical TeleopInput
  -> latest-only runtime
  -> MockRobotBackend
```

It includes strict packet codecs, golden vectors, sequence/session handling,
latest-only UDP, low-rate clock sync, packet metrics, network tools, and local
integration tests. It does not include Swift/iOS code, ARX5, CAN, IK, Ruckig,
visualization, or hardware motion.

## Requirements

### Functional

- Encode and decode Pose, Ping, and Pong messages with one versioned envelope.
- Convert valid ARKit pose messages into canonical `TeleopInput` values.
- Reject malformed, unsupported, non-finite, inconsistent, duplicate,
  out-of-order, retired-session, and implausible-jump inputs.
- Respond to Ping immediately without acknowledging Pose messages.
- Preserve latest-only semantics across both the UDP socket and runtime handoff.
- Expose loss, rejection, packet-age, RTT, and control-loop metrics.
- Run ARKit-protocol UDP to MockRobot without any robot SDK.

### Non-functional

- Expected Pose rate is 60 Hz; the network path must remain comfortably above
  that rate without queue growth.
- The decoder uses precompiled `struct.Struct` objects and no JSON, reflection,
  dynamic schema, or per-packet logging.
- UDP bursts are drained with a fixed upper bound and superseded datagrams are
  discarded before semantic processing.
- Codec and loopback tools report P50/P95/P99; CI validates behavior rather than
  fragile wall-clock thresholds.
- All workers and sockets stop explicitly within bounded time.
- Phase 1 remains dry-run and hardware-free.

## Wire protocol v1.0

The approved v1 envelope has a 36-byte common header:

| Offset | Type | Field |
|---:|---|---|
| 0 | `UInt32` | magic, wire bytes `RTLP` |
| 4 | `UInt16` | major version |
| 6 | `UInt16` | minor version |
| 8 | `UInt16` | packet size |
| 10 | `UInt16` | packet type |
| 12 | `UInt16` | source type |
| 14 | `UInt16` | reserved, zero in v1.0 |
| 16 | `UInt32` | flags |
| 20 | `UInt32` | sequence |
| 24 | `UInt32` | session ID |
| 28 | `UInt64` | client time or echoed client time |

Formats are little-endian and fixed-length:

| Message | Type | Size | Payload |
|---|---:|---:|---|
| Pose | 1 | 96 bytes | `14f`, tracking byte, three reserved bytes |
| Ping | 2 | 36 bytes | none; header time is client send `t0` |
| Pong | 3 | 52 bytes | server receive `t1`, server send `t2` |

Pose retains position, `xyzw` quaternion, optional linear/angular velocity,
optional gripper, tracking state, and the existing flags. Four bytes added to
the Phase 0 draft buy explicit dispatch and natural 4-byte alignment. Golden
vectors freeze this layout for Phase 2 Swift compatibility.

Stable source values reserve `0` for unknown/server and assign ARKit `1`, with
future values for VR, SpaceMouse, hand tracking, teaching, replay, and mock.
Only ARKit is assembled as a live Phase 1 source.

## Validation and ordering

Validation occurs before canonical publication:

1. Read only the common header and verify magic, major, declared and actual size.
2. Dispatch by known packet type and require its exact fixed size.
3. Validate source type, v1.0 reserved bytes, flags, enums, and all float values.
4. Require raw quaternion norm within 5 percent before canonical normalization.
5. Require flag/tracking/capability consistency and gripper range.
6. Apply per-session sequence and timestamp checks.
7. Apply canonical position/rotation jump checks in the existing receiver.

Sequence comparison uses unsigned serial arithmetic. A forward delta in
`[1, 2^31)` is new and supports wrap; zero is duplicate; the other half-range is
old/out of order. A session switch retires the previous ID in a bounded set so a
delayed old-session datagram cannot switch control back. A sequence gap records
estimated loss but accepts the newest valid packet.

## Latest-only UDP

`UdpTransport` remains a byte transport. One blocking `recvfrom` obtains a
datagram, then a bounded non-blocking drain discards older queued datagrams and
returns only the newest. This keeps kernel backlog from becoming command age.
The number of received and superseded datagrams is recorded.

Dropping a low-rate Ping during a Pose burst is acceptable because Ping is
periodic and UDP is already lossy. Pong is sent immediately to the peer of the
last valid Ping. The transport supports an optional configured remote peer for
test and client tools. Socket timeout and close both unblock shutdown.

Alternatives rejected:

- Preserve all datagrams: creates obsolete work and growing command age.
- Decode robot semantics in transport: violates layer ownership.
- Separate clock-sync socket: adds client lifecycle and firewall complexity.
- Protobuf/JSON: adds size, allocation, dependency, and cross-language ambiguity
  without useful v1 flexibility.

## Clock synchronization

The client records `t0`, the server records `t1` and `t2`, and the client records
`t3`. The estimator calculates:

```text
RTT = (t3 - t0) - (t2 - t1)
server_minus_client = ((t1 - t0) + (t2 - t3)) / 2
```

Samples with negative intervals or excessive RTT are rejected. The estimate is
the median offset of a bounded set of the lowest-RTT samples. Pose packets are
never acknowledged. Until a sync estimate exists, Ubuntu reports host receipt
age rather than pretending raw client/server monotonic subtraction is one-way
latency.

## Metrics

`RuntimeMetrics` remains the single report owner. A generic protocol event
method records datagrams, superseded datagrams, decoded packet types, rejects by
reason, sequence gaps, retired sessions, Ping/Pong, and session changes. Bounded
sample deques hold RTT and codec/network latency values. No worker writes files
or formats high-rate output.

## Failure modes

| Failure | Behavior |
|---|---|
| Wrong magic/version/size/type | reject and count; receiver continues |
| NaN/Inf/bad quaternion/flags | reject and count; receiver continues |
| Duplicate/out-of-order packet | reject and count |
| Sequence gap | accept newest, count estimated loss |
| Delayed prior session | reject retired session |
| UDP burst | discard superseded datagrams, publish newest |
| No packets | existing Watchdog hold/brake/safe-stop path |
| Ping/Pong loss | next low-rate Ping retries; Pose is unaffected |
| Socket close during receive | clean worker exit |
| Receiver exception | record fault; Watchdog stops Mock backend |
| Untrusted LAN traffic | strict validation; deployment remains trusted-LAN only |

UDP has no authentication in Phase 1. Documentation must prohibit internet
exposure and recommend an isolated/trusted control network. An optional allowed
peer may be configured later without changing core.

## Verification

- Exact golden bytes for Pose, Ping, and Pong.
- Mutation tests for every header and payload rejection class.
- Sequence wrap, gap, duplicate, out-of-order, session switch, and retired ID.
- Clock offset/RTT, asymmetric delay sensitivity, outlier rejection, and bounds.
- UDP lifecycle, send/receive, latest burst, timeout, and clean close.
- ARKit adapter flags/capabilities/tracking and Ping response.
- UDP ARKit-to-Mock integration, network loss, session restart, and no backlog.
- Tool smoke tests and a local latency/packet-loss report.
- Full Ruff, format, strict mypy, pytest, and architecture checks.

