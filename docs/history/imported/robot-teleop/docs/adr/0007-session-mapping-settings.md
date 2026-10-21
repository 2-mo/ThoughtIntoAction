# ADR-0007: Apply session mapping settings outside the input adapter

## Status

Accepted

## Context

The iOS requirements include position and rotation sensitivity controls. The
v1 Pose packet deliberately contains only source motion and control state.
Applying sensitivity to ARKit world poses on the phone would duplicate relative
mapping, make behavior device-specific, and make orientation scaling depend on
the input adapter.

Changing an active mapping immediately can also jump the current target because
the same input delta would be recomputed with a different gain.

## Decision

Extend the fixed v1 message registry with 44-byte Settings and SettingsAck
messages. Settings carries finite non-negative position scale and a finite
rotation scale in `[0, 1]`. It is low-rate, uses no Pose ACK, and is accepted
only for the session established by the latest accepted Pose.

The ARKit adapter converts the packet to canonical `MappingScales` and sends it
to an injected callback. Runtime owns a thread-safe `MappingSettingsStore`.
`RelativePosePolicy` reads the store only during reset, which occurs on Clutch
activation or recenter, and retains that immutable mapping for the active
segment. SettingsAck echoes the values stored so the client can distinguish
pending from applied controls.

Pose, Ping, and Settings use independent sequence streams. Ping and Settings
sequences are correlation/order values and do not contribute to Pose loss.

## Consequences

### Positive

- Mapping and scale remain device-independent and implemented once.
- Slider changes cannot alter an already active target segment.
- The app receives explicit evidence that Ubuntu applied its low-rate setting.
- Robot and common target limits remain authoritative after scaling.

### Negative

- The protocol gains two message types and two additional golden vectors.
- A Phase 1 receiver will reject Settings as unknown, although unchanged Pose
  streaming continues; the app must show settings as unacknowledged.
- Settings must follow the first released Pose of a new session.

### Neutral

- Position scale has no arbitrary protocol maximum. Deployments choose a useful
  UI range, while existing target and RobotBackend safety limits still bound
  commands.

## Alternatives considered

- Scale ARKit poses in the app: rejected because it duplicates canonical mapping
  and makes orientation behavior source-specific.
- Put scales into every Pose: rejected because the values are low-rate and would
  enlarge the hot packet permanently.
- Mutate the active policy mapping immediately: rejected because it can create a
  target discontinuity.
- Keep visible sliders that do nothing: rejected because operational controls
  must be truthful.

## References

- [`../plans/2026-08-03-phase-2-design.md`](../plans/2026-08-03-phase-2-design.md)
- [`../protocol.md`](../protocol.md)
