# ADR-0001: Pure canonical core

## Status

Accepted

## Context

Input devices, transports, policies, controllers, and robots must be replaceable
without importing vendor SDKs or runtime I/O into shared behavior.

## Decision

Keep canonical immutable models, frame math, session/state behavior, shared
safety, and port protocols in `robot_teleop.core`. Core uses only the Python
standard library. Concrete packages depend on core, never the reverse.

## Consequences

### Positive

- Core tests run without ARX5, Xcode, ROS, sockets, or URDF assets.
- Vendor ordering and units are converted at one boundary.
- Architecture rules can be checked statically.

### Negative

- Core geometry implements a small, reviewed quaternion/matrix subset.
- Adapter base modules re-export protocols rather than owning them.

## Alternatives considered

- NumPy in core: rejected to keep the dependency and type surface small.
- Vendor-native data classes: rejected because they leak backend concepts.

