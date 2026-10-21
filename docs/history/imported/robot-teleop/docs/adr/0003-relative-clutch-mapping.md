# ADR-0003: Relative clutch mapping

## Status

Accepted

## Context

Mapping an input world's absolute pose into a robot base creates jumps after
recenter, reconnect, tracking recovery, and clutch cycles.

## Decision

On activation capture `T_input_reference` and `T_robot_reference`. Compute
`inverse(T_input_reference) * T_input_current`, apply signed axis and scale
mapping, then compose it with `T_robot_reference`. Session logic owns all clutch
and recenter edges.

## Consequences

### Positive

- First activation and recenter produce a zero delta.
- Input world origins never become robot base origins.
- One policy supports device-independent 6DoF relative control.

### Negative

- Axis mappings, including reflections, require matrix conjugation for rotation.
- Mapping must be verified with tests and later Ghost visualization.

## Alternatives considered

- Absolute pose mapping: rejected due to discontinuities and unsafe assumptions.
- Device-specific clutch code: rejected because it duplicates continuity logic.

