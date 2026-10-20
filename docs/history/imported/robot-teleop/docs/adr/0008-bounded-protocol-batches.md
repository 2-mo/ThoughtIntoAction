# ADR-0008: Preserve control datagrams while coalescing Pose

## Status

Accepted

## Context

The original UDP receive path drained a bounded kernel backlog and returned only
the final datagram. That is correct for a Pose-only stream, but protocol v1 also
shares the endpoint with Ping and Settings. A live iOS Simulator run showed a
timer-aligned Ping could repeatedly be followed by a Pose and discarded before
decode. Sending controls after Pose made controls reliable but introduced a
periodic missing Pose and a roughly 33 ms interarrival gap.

The system needs both properties: obsolete Pose targets must never accumulate,
and low-rate control packets must not be discarded merely because a newer Pose
arrived in the same bounded socket drain.

## Decision

`UdpTransport` exposes an additive `receive_batch()` operation. It performs the
same one blocking receive plus bounded nonblocking drain and returns at most the
configured finite batch. Its existing `receive()` operation remains compatible
and still returns only the newest datagram.

`ArkitInputSource` detects batch-capable transports structurally. It decodes the
finite batch in arrival order, handles every valid Ping and Settings packet, and
returns only the newest valid `TeleopInput`. Older accepted Pose packets in that
same batch are counted as superseded but are never published to the runtime.
Sequence/session validation still observes every decoded Pose, so control
packets can follow the session-establishing Pose in the same batch.

The iOS coordinator schedules at most one low-rate control packet after a Pose.
This improves wire ordering without pausing the pose provider or creating a
reliable historical queue. Timeout/retry remains bounded and Pose remains
unacknowledged.

## Consequences

### Positive

- Ping and Settings remain live during a continuous 60 Hz Pose stream.
- Control delivery no longer requires sacrificing a Pose frame.
- Only one newest Pose reaches `LatestValue`; command latency cannot grow by
  replaying the bounded batch.
- `core/` and the generic `Transport.receive()` contract remain unchanged.

### Negative

- The ARKit adapter may decode up to the configured finite drain bound in one
  receiver iteration.
- A transport used through its generic single-message method still coalesces all
  byte messages indiscriminately; mixed protocol users must opt into the batch
  method at their decoder boundary.

### Neutral

- UDP still provides no delivery, ordering, authentication, or encryption
  guarantee. Ping and Settings remain matched, low-rate, and retry tolerant.

## Alternatives considered

- Use a reliable queue for all messages: rejected because historical Pose has no
  control value and would add latency.
- Put control traffic on a second UDP port: rejected because it complicates
  discovery, firewall setup, and session ownership for no current benefit.
- Pause Pose around each control send: rejected because it creates periodic
  source gaps and depends on network timing.
- Classify packets inside `UdpTransport`: rejected because byte transport must
  not own protocol or robot semantics.

## References

- [`0006-versioned-udp-message-envelope.md`](0006-versioned-udp-message-envelope.md)
- [`../architecture.md`](../architecture.md)
- [`../performance.md`](../performance.md)
- [`../phase2_report.md`](../phase2_report.md)
