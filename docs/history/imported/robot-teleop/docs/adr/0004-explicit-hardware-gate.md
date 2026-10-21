# ADR-0004: Explicit hardware gate

## Status

Accepted

## Context

Configuration files are easy to reuse or edit accidentally. A real backend must
never connect merely because a profile contains `hardware.enabled: true`.

## Decision

Phase 0 registers no hardware backend. Future non-mock backends require the
runtime profile and a separate explicit `--enable-hardware` CLI flag. Backend
construction happens only after this check.

## Consequences

### Positive

- Default and CI paths cannot connect to a robot.
- A copied configuration alone cannot authorize hardware.

### Negative

- Hardware runs require an intentionally verbose command.

## Alternatives considered

- Configuration-only enablement: rejected as too easy to trigger accidentally.
- Environment-variable enablement: rejected because inherited shells are not an
  explicit operator action at command invocation.

