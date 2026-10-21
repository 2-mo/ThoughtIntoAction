# ADR-0005: Optimize responsiveness within hard safety bounds

## Status

Accepted

## Context

Teleoperation quality depends on low input-to-command latency, low Servo jitter,
and continuous motion. Treating every initial Watchdog, filtering, control-rate,
and recovery choice as a permanent safety constraint would make the system
conservative in ways that can add phase lag, stop-start behavior, and poor clutch
interaction without proving a safer physical outcome.

The framework still needs a narrow set of boundaries that prevent unintended
hardware activation and commands outside verified robot capability.

## Decision

Separate requirements into three groups:

1. Hard boundaries: explicit hardware enablement, latched E-stop, truthful
   physical E-stop documentation, RobotBackend hard limits, and no automated
   robot motion from CI or Codex.
2. Performance invariants: latest-only commands, no avoidable blocking work on
   the Servo path, bypassable filters, and end-to-end latency/continuity metrics.
3. Reviewable policies: control frequency, Watchdog thresholds, filter and
   prediction parameters, recovery interaction, common limits within the backend
   envelope, process/thread placement, and Python/C++ ownership.

Optimize the reviewable policies aggressively using P50/P95/P99 latency, jitter,
deadline misses, target/velocity/acceleration continuity, and measured stopping
behavior. Phase 0 values are safe starting points, not permanent performance
targets. An architectural change requires benchmark evidence, failure-path tests,
and an ADR update.

## Consequences

### Positive

- Responsiveness and smoothness are considered in every phase rather than after
  the hardware path is complete.
- Conservative defaults can be relaxed when evidence shows a faster bounded
  configuration.
- Hard robot protection remains independent of frontend and network tuning.
- Filters and recovery behavior must justify their phase delay and interaction
  cost.

### Negative

- More configurations and network conditions must be benchmarked.
- Tuning cannot rely on a single universal set of timeout or filter values.
- Comparing smoothness requires trajectory-continuity metrics in addition to
  loop-frequency statistics.

### Neutral

- The current Phase 0 implementation and defaults remain unchanged until a later
  phase supplies the measurements and tests needed to tune them.

## Alternatives considered

- Fix every conservative default permanently: rejected because it can trade away
  control quality without evidence of improved stopping behavior.
- Make performance the only constraint: rejected because hardware activation,
  E-stop, and backend hard limits need independent protection.
- Tune by operator feel alone: rejected because subjective feel is valuable but
  must be paired with repeatable latency, continuity, and stop measurements.

## References

- [`../performance.md`](../performance.md)
- [`../safety.md`](../safety.md)
- [`../../TODO.md`](../../TODO.md)
