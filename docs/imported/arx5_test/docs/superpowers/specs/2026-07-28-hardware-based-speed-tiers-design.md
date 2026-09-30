# Hardware-Based Speed Tiers Design

## Status

Approved on 2026-07-28.

## Goal

Define the web console's `100%` tier as the X5 SDK hardware velocity ceiling
and derive every lower tier from that same full-speed trajectory. The console
opens at `10%` by default.

## Motion Profile

The web service exposes one motion profile. There is no separate safe/hardware
mode and no confirmation toggle.

- Joint velocity uses the SDK per-joint limits:
  `[5.0, 5.0, 5.5, 5.5, 5.0, 5.0] rad/s`.
- Cartesian translation uses the SDK ceiling of `0.6 m/s`.
- Gripper translation uses the SDK ceiling of `0.3 m/s`.
- Joint acceleration and jerk remain the established hardware-profile values
  of `4.0 rad/s^2` and `24.0 rad/s^3` because the SDK does not publish those
  derivative limits.
- Cartesian acceleration and jerk use `1.2 m/s^2` and `12.0 m/s^3`.
- The minimum full-speed motion duration is `0.25 s`.

The scalar application joint velocity ceiling is `5.5 rad/s`; controller
planning and validation continue to intersect it with `RobotConfig` so each
joint receives its own SDK limit.

## Tier Semantics

The available tiers remain:

`1%, 2%, 3%, 4%, 5%, 10%, 25%, 50%, 75%, 100%`.

For a tier scale `s`, geometry and IK come from the canonical 100% path. Motion
time scales by `1 / s`, command velocity by `s`, acceleration by `s^2`, and
jerk by `s^3`. Selecting a tier must not switch profiles or solve a different
geometric path.

Both the browser slider and server fallback default to `10%`, so requests that
omit `speed_percent` behave exactly like the initial UI.

## Compatibility

Legacy `speed_profile` and `hardware_max_confirmed` request fields remain
rejected. The protocol continues to carry only `speed_percent`.

Existing joint limits, IK/FK checks, trajectory validation, feedback monitoring,
torque checks, and emergency damping behavior are unchanged.

## Verification

- Contract tests assert the 100% joint, Cartesian, gripper, acceleration, jerk,
  and minimum-duration values.
- Browser and server tests assert a default tier of 10%.
- Duration tests assert 100%, 50%, and 25% are `1x`, `2x`, and `4x` versions of
  the same canonical path.
- The full offline test suite and JavaScript syntax checks run before handoff.
