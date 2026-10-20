# ADR-0006: Use a 96-byte versioned UDP pose message

## Status

Accepted. The receive-coalescing detail is amended by ADR-0008.

## Context

The Phase 0 draft defined a 92-byte Pose packet but had no packet-type field.
Phase 1 needs Pose, Ping, and Pong messages on one UDP endpoint while preserving
fixed-size little-endian parsing and future device independence.

## Decision

Use a 36-byte common header with explicit packet type and source type. Define
fixed v1.0 messages: 96-byte Pose, 36-byte Ping, and 52-byte Pong. Use precompiled
Python `struct.Struct` codecs and publish exact golden binaries. The golden
vectors freeze v1.0 for the Phase 2 Swift implementation.

UDP reception is bounded and Pose remains latest-only. ADR-0008 refines batch
handling so protocol controls are processed while only the newest Pose is
published. Pose has no acknowledgement; low-rate Ping/Pong provides clock
samples.

## Consequences

### Positive

- One endpoint dispatches all v1 messages without magic or size overloading.
- Four-byte alignment and fixed sizes make Swift/Python implementation direct.
- Latest-only draining prevents socket backlog from becoming motion latency.
- Future input devices retain a stable source registry.

### Negative

- The Phase 0 draft changes from 92 to 96 bytes before its first external use.
- UDP provides no delivery, ordering, or authentication guarantee.
- Control delivery is still best-effort and uses bounded timeout/retry behavior.

### Neutral

- Phase 1 remains compatible only with its checked-in golden v1 vectors; no
  deployed v1 sender exists yet.

## Alternatives considered

- Keep 92-byte Pose and use a second magic for clock sync: rejected because it
  fragments versioning and dispatch.
- Dispatch solely by packet size: rejected because size is not semantic identity.
- Protobuf, FlatBuffers, or JSON: rejected for v1 due to extra dependencies,
  allocation, and cross-language schema complexity.
- Reliable transport or per-Pose ACK: rejected because retransmitted old poses
  have no live control value.

## References

- [`../plans/2026-08-03-phase-1-design.md`](../plans/2026-08-03-phase-1-design.md)
- [`../protocol.md`](../protocol.md)
- [`0008-bounded-protocol-batches.md`](0008-bounded-protocol-batches.md)
