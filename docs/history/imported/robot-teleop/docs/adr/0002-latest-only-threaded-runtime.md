# ADR-0002: Latest-only threaded runtime

## Status

Accepted

## Context

Old teleoperation targets have no execution value. The control loop needs clear
deadline ownership and must not accumulate network jitter as queued work.

## Decision

Use receiver, servo, and metrics threads. Receiver-to-servo handoff is a locked,
overwrite-only `LatestValue`. The servo skips missed history and rebases when
more than two periods late.

## Consequences

### Positive

- Memory is constant and command age cannot grow through queue backlog.
- Blocking input does not execute on the servo thread.
- Metrics and output formatting stay outside the servo thread.

### Negative

- Python thread jitter must be measured before claiming real-time behavior.
- Components need explicit, tested start/stop ordering.

## Alternatives considered

- FIFO queue: rejected because it preserves obsolete motion commands.
- asyncio-only runtime: rejected because future robot SDK calls are synchronous.
- multi-process core: deferred until measurements justify IPC complexity.

