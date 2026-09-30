# Coordinate Frames

## Conventions

- Distance: metres.
- Time: seconds or explicitly suffixed nanoseconds.
- Angles and angular velocity: radians and radians/second.
- Quaternion field order: `x, y, z, w` everywhere in canonical code.
- Pose transform: `T_parent_child`, mapping child coordinates into parent.
- Composition: `compose(A, B)` means matrix product `A * B`.
- Rotation deltas use the shortest quaternion path.

Vendor backends convert other quaternion orders exactly once at their boundary.
No Euler-angle filtering is permitted.

## Relative clutch mapping

On clutch activation or recenter, capture:

```text
T_input_reference
T_robot_reference
```

For a later input sample:

```text
T_input_delta = inverse(T_input_reference) * T_input_current
T_mapped_delta = MapAndScale(T_input_delta)
T_robot_target = T_robot_reference * T_mapped_delta
```

This deliberately does not identify ARKit world with robot base.

## Signed axis mapping

Position mapping is a signed permutation. The default ARKit-to-gripper profile is
human-aligned for a phone lying face-up in portrait orientation:

- phone top edge / forward = gripper forward (`eef_link +X`)
- phone right edge = gripper right (`eef_link -Y`, because `+Y` is gripper left)
- screen normal / face-up = gripper up (`eef_link +Z`)

```text
robot_x = +input_y
robot_y = -input_x
robot_z = +input_z
```

For rotation, let `M` be the signed permutation matrix and `R_delta` the input
delta rotation. Mapping uses:

```text
R_mapped = M * R_delta * transpose(M)
```

This remains a proper rotation even when `M` includes a reflection. Position
scale multiplies the mapped translation. Rotation scale interpolates from the
identity quaternion to the mapped delta using shortest-path SLERP.

## Continuity invariants

The first target after activation, recenter, new session, tracking recovery, or
backend reconnect equals the current robot pose. A held clutch across a session
change or tracking recovery cannot reactivate control until released. Clutch
release produces HOLD and clears active references.

Axis direction and orientation must be confirmed with unit tests, replay, and a
Ghost visualization before any hardware test.
