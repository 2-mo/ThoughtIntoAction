# ADR-0009: Require local completion for E-stop reset

## Status

Accepted

## Context

An E-stop request is transmitted by a remote input device and latches the
canonical pipeline and backend stop path. The existing phone-local latch can
only be cleared by restarting the app, which is unsuitable for a future
operational workflow. A phone-only clear would make the interface appear ready
while Ubuntu might remain in `ESTOP`. Conversely, clearing Ubuntu as soon as a
network reset packet arrives would let network traffic clear a safety latch.

Phase 3 has only `MockRobotBackend`; no real backend exposes a verified recovery
operation. The decision must therefore preserve the generic safety boundary and
avoid implying that an ARX5 can be re-armed this way.

## Decision

Add fixed-size, session-scoped `ESTOP_RESET_REQUEST` and `ESTOP_RESET_ACK`
messages in protocol v1.1. The iPhone sends a request and a later Clutch-released
Pose with its wire E-stop bit clear, while retaining the visible UI lock.

Ubuntu records the request but never clears its canonical E-stop automatically.
Only `TeleopApplication.reset_estop()`, called by a local operator, may act. It
requires a matching active-session request and a later valid normal-tracking
released Pose. In Phase 3 it invokes a Mock-only stationary re-arm callback,
resets the pipeline, restarts the dry-run Servo, and then sends the matching
acknowledgement. The phone unlocks only after validating that acknowledgement.

The reset gate lives in `runtime/`; `core/` remains unaware of packet types,
networking, and any specific input or robot. The registry injects the Mock
re-arm callback rather than teaching the generic robot port a hardware recovery
method.

## Consequences

### Positive

- Neither endpoint can independently make the system look or behave reset.
- The phone remains locked while Ubuntu is still latched.
- A fresh, neutral release state is required before the next Clutch activation.
- Phase 3 tests can prove the protocol and lifecycle without hardware support.

### Negative

- The operator must perform a local explicit action for every reset.
- Reset requires a future verified backend-specific recovery design before a
  hardware backend can participate.
- A lost acknowledgement can leave the phone locked even after a completed Mock
  reset; the completed request must remain available for idempotent re-ack.

### Neutral

- The new messages are low-rate control traffic and do not acknowledge or queue
  high-rate Pose packets.

## Alternatives considered

- Phone-local reset only: rejected because it can diverge from Ubuntu's latch.
- Clear Ubuntu on ResetRequest: rejected because it converts a remote packet
  into reset authority.
- Add an HTTP reset endpoint: rejected because it adds network authority and a
  service dependency before it is needed.
- Add a generic backend `recover()` method now: rejected because no ARX5 recovery
  semantics have been verified yet.

## References

- [`../plans/2026-08-04-phase-3-mock-validation-design.md`](../plans/2026-08-04-phase-3-mock-validation-design.md)
- [`../safety.md`](../safety.md)
- [`0002-latest-only-threaded-runtime.md`](0002-latest-only-threaded-runtime.md)
