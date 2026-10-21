# Reference Review

Review date: 2026-08-03. Every SHA below is the checked-out HEAD inspected for
Phase 0. Source was read from shallow clones in a temporary directory. No source
code was copied or modified from any reference project.

## Comparison

| Project | Adopted | Explicitly not adopted |
|---|---|---|
| `2-mo/arx5_test` | latest generation, fixed-period servo, online replan, stale-feedback stop, deadline rebase, layered safety | Web/API stack, cameras, perception, experiments, compatibility layers |
| `real-stanford/arx5-sdk` | future backend boundary, state/config/limit inventory, damping primitive | sample-script architecture, automatic home/reset, direct device-input coupling |
| `yihuai-gao/iPhoneVIO` | one send per `ARFrame`, world-tracking setup, camera capability/permission pattern | Socket.IO, base64, per-frame printing, sleeps, 4x4 payload, missing tracking-state handling |
| `XRoboToolkit-Teleop-Sample-Python` | clutch reference capture, relative pose, signed headset/world mapping, gripper abstraction | PICO service, Placo main path, cameras, dual-arm assumptions, hardware initialization in constructors |
| `pantor/ruckig` | future online update/pass-to-input loop, limit validation, velocity-mode braking | reimplemented S-curves, per-packet full trajectories, Pro-only tracking assumption |
| `stack-of-tasks/pinocchio` | future URDF/FK/frame/Jacobian APIs and singularity diagnostics | full dynamics stack in Phase 0, unbounded iterative IK on the servo thread |

## 2-mo/arx5_test

- Repository: https://github.com/2-mo/arx5_test
- Commit SHA: `0dbffd38bd4032df80c0913bfabf821e7c8c2175`
- License: No repository-level license file was present at this SHA. Asset-level
  licenses do not license the project source.
- Files inspected: `src/arx5_control/teleoperation_servo.py`,
  `src/arx5_control/servo.py`, `src/arx5_control/motion/online.py`,
  `src/arx5_control/teleoperation.py`, `src/arx5_control/cartesian.py`,
  `src/arx5_control/motion/pinocchio_backend.py`,
  `src/arx5_control/motion/jacobian.py`,
  `src/arx5_control/controller.py`,
  `src/backend/arx5_backend/kernel/control.py`, and
  `configs/teleoperation.json`.
- Relevant behavior: latest target plus monotonic generation, fixed-period
  online Ruckig stepping, joint/Cartesian safety intersection, workspace and
  singularity speed scaling, feedback timestamp progress checks, watchdog
  braking, deadline rebase, and emergency damping on worker failure.
- Design adopted: generation-aware latest target, no catch-up after large
  deadline misses, separate stale-feedback check, backend emergency stop on
  runtime failure, discontinuous IK branch rejection, and a generic
  controller-independent Cartesian-to-Ruckig composition for offline preflight.
- Offline-profile evidence: the local reference's `fast` profile uses a
  `3.5 rad/s` joint velocity value, `0.70 rad/s^2` acceleration, and
  `3.0 rad/s^3` jerk. This project labels those values as benchmark inputs only;
  it does not treat them as physical-arm limits.
- Design rejected: browser/WebSocket/FastAPI ownership, MediaPipe/RealSense,
  calibration and grasping flows, long application kernel, and any assumption
  that a browser command is a canonical teleoperation input.
- Code copied or modified: None.
- Attribution required: None for Phase 0 because no code was copied. License
  status must be resolved before any future reuse beyond behavioral study.
- Relevant tests: `tests/test_cartesian_path_quality.py` checks IK path branch
  continuity; `tests/test_web_motion_limits.py` checks trusted motion limits.

## real-stanford/arx5-sdk

- Repository: https://github.com/real-stanford/arx5-sdk
- Commit SHA: `8ccbf0aa8d4288f2721c41a735e5ac4c389bb13b`
- License: MIT, copyright 2024 Yihuai Gao.
- Files inspected: `python/arx5_interface.pyi`, `python/arx5_pybind.cpp`,
  `include/app/controller_base.h`, `include/app/cartesian_controller.h`,
  `include/app/joint_controller.h`, `include/app/config.h`,
  `include/app/solver.h`, `models/X5.urdf`,
  `python/examples/spacemouse_teleop.py`,
  `python/examples/joint_waypoint_scheduling.py`,
  `python/examples/cartesian_waypoint_scheduling.py`, `README.md`.
- Relevant behavior: config factories expose joint position/velocity/torque and
  end-effector/gripper limits; joint and Cartesian controllers expose state,
  command, timestamp, gain, reset, and damping APIs; `Arx5Solver` exposes FK and
  IK. The X5 URDF has a six-joint `base_link` to `eef_link` chain. Its generic
  URDF limits differ from the SDK's runtime X5 limits, so offline checks must
  explicitly use the cited SDK configuration rather than blindly trust URDF
  limits. The SDK documents a 500 Hz background joint controller and warns that
  its default checks are insufficient near singularities or noisy inputs.
- Design adopted: keep SDK lifecycle and all `xyzw`/SDK conversion in a future
  backend; intersect frontend requests with SDK/backend hard limits; treat
  `set_to_damping` as a robot-specific stop primitive to verify on hardware.
- Design rejected: calling `reset_to_home()` on construction, using the
  SpaceMouse example as application architecture, busy-wait timing, per-cycle
  printing, and sending scheduled waypoints for every live input update.
- Code copied or modified: `models/X5.urdf` was copied to
  `assets/robots/arx5/X5.urdf` for Phase 4 offline model checks; its only
  change is trailing-whitespace normalization. No SDK source code was copied.
- Attribution required: retain the MIT attribution in
  `assets/robots/arx5/README.md` and this record when distributing the copied
  model asset.
- Relevant tests: `tests/unit/test_kinematics.py` validates the attributed model
  metadata and deterministic relative target construction;
  `tests/integration/test_phase4_pinocchio.py` validates the optional model
  loader, FK, seeded IK, Jacobian, and report export.

## Local Phase 4/5 Review: 2-mo/arx5_test

- Repository: local reference checkout at `/home/r2/Desktop/arx5_test` on the
  Ubuntu host; no files were imported into this repository.
- Commit SHA: `69a144f1cd7b6157a536428096b1f52e25d9200e` (the inspected
  worktree was clean and locally ahead of its remote by ten commits).
- License: this local reference remains subject to the Phase 0 repository-level
  license caveat; it was studied for behavior only.
- Files inspected: `src/arx5_control/motion/pinocchio_backend.py`,
  `src/arx5_control/motion/jacobian.py`,
  `src/arx5_control/cartesian.py`,
  `src/arx5_control/teleoperation_servo.py`,
  `src/arx5_control/controller.py`,
  `src/arx5_control/motion/online.py`,
  `src/backend/arx5_backend/kernel/control.py`,
  `configs/teleoperation.json`,
  `calibrations/gripper/calibration.json`, `docs/CONTROL_TERMINAL.md`,
  `docs/TRAJECTORY_CONTROL.md`, and `docs/SAFETY.md`.
- Relevant behavior: Pinocchio model/data allocation occurs outside the live
  path; frame Jacobians are evaluated in an explicit reference frame; the prior
  IK solution is used as a continuity seed; singularity metrics scale rotational
  rows by a characteristic length; online Ruckig state is separate from
  offline/preview checks; teleoperation uses latest target generation,
  watchdog-triggered braking, workspace speed scaling, and verified damping/hold
  transitions in the application-specific controller.
- Design adopted: optional import isolation, a fixed model/data lifetime,
  Jacobian diagnostics, bounded seeded offline IK preflight, and treating
  reference values such as `can0`, X5, workspace bounds, velocity profiles, and
  gripper readout as evidence to verify rather than hard-coded truths.
- Design rejected: importing its controller, SDK solver, CAN lifecycle,
  Ruckig executor, web API, calibration, collision defaults, or robot-specific
  safety constants into this generic framework.
- Code copied or modified: None.
- Attribution required: None, because no local-reference code or assets were
  copied.
- Relevant tests: this repository's `test_phase4_pinocchio.py` covers the
  equivalent offline boundary with a source-attributed upstream X5 URDF.

## Phase 5 Package Boundary: arx5-interface

- Package: `arx5-interface==0.1.3` from PyPI.
- Distribution evidence: the installed Ubuntu wheel declares MIT in package
  metadata and is tagged CPython 3.12, manylinux x86_64. It has no macOS wheel,
  so the dependency is marker-scoped to Linux x86_64 in this project.
- Static package inspection on Ubuntu: X5 reports 6 joints, `base_link` to
  `eef_link`, a `0.002 s` controller period, and the same position/velocity
  vectors held in `configs/robots/arx5.yaml`. It also reports default background
  I/O enabled and linear interpolation; the framework does not adopt either
  unreviewed default. No controller was constructed in this inspection.
- Upstream repository: https://github.com/real-stanford/arx5-sdk at
  `8ccbf0aa8d4288f2721c41a735e5ac4c389bb13b` (MIT).
- Files and metadata inspected: upstream `python/arx5_interface.pyi`, package
  `METADATA` and `WHEEL`, and the local reference's SDK construction/state
  conversion/Ruckig files listed above.
- Relevant behavior: `Arx5JointController` construction takes a model,
  controller config, and CAN interface; state exposes joint arrays plus a
  `pose_6d` EEF layout; the inspected reference treats that layout as
  `XYZ + fixed-axis RPY`, but derives measured TCP feedback from measured joint
  positions through FK; `set_to_damping()` is the documented stop candidate.
- Design adopted: lazy import, no construction without both explicit hardware
  and interface gates, measured-joint FK as a required runtime input, canonical
  `xyzw` conversion only at the backend, fake SDK contracts, and a generic
  upstream Ruckig wrapper.
- Design rejected: SDK construction in module import/default CLI/CI, automatic
  homing, background SDK I/O ownership, raw Cartesian forwarding, unverified
  gripper normalization, and a custom S-curve.
- Code copied or modified: None.
- Attribution required: retain upstream/package MIT attribution if SDK binaries
  or source are redistributed; the project does not vendor either.

## yihuai-gao/iPhoneVIO

- Repository: https://github.com/yihuai-gao/iPhoneVIO
- Commit SHA: `adabceaf02f0485a64c0f739f04c1d945b3802a4`
- License: No repository-level license file was present. Vendored dependencies
  have their own licenses and do not license the application source.
- Files inspected: `iPhoneVIO/ARSessionManager.swift`,
  `iPhoneVIO/SocketClient.swift`, `iPhoneVIO/ARManager.swift`,
  `iPhoneVIO/ARAction.swift`, `iPhoneVIO/ContentView.swift`,
  `iPhoneVIO.xcodeproj/project.pbxproj`,
  `iPhoneVIOTests/iPhoneVIOTests.swift`.
- Relevant behavior: `ARWorldTrackingConfiguration`, `ARSessionDelegate`, and
  one pose emission from each `session(_:didUpdate:)` callback; generated
  Info.plist settings require ARKit and a camera usage description.
- Design adopted: capture on ARFrame callbacks, never repeat an old pose with a
  high-rate timer, and keep host/port plus tracking presentation in the app.
- Design rejected: Socket.IO/base64, a native 4x4 matrix stream, blocking
  `usleep`, forced unwraps, per-frame prints, and ignoring ARKit tracking state.
- Code copied or modified: None.
- Attribution required: None for Phase 0. Do not copy source unless licensing is
  clarified.
- Relevant tests: The XCTest target contains only generated placeholder tests;
  it has no binary or ARKit behavior assertions.

## XR-Robotics/XRoboToolkit-Teleop-Sample-Python

- Repository: https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python
- Commit SHA: `79e5cb8a56e3455515ce1b476e993c764ec58739`
- License: MIT, copyright 2025 XR Robotics.
- Files inspected: `xrobotoolkit_teleop/common/base_teleop_controller.py`,
  `xrobotoolkit_teleop/common/base_hardware_teleop_controller.py`,
  `xrobotoolkit_teleop/utils/geometry.py`,
  `xrobotoolkit_teleop/hardware/dual_arm_ur_controller.py`,
  `xrobotoolkit_teleop/hardware/arx_r5_teleop_controller.py`,
  `xrobotoolkit_teleop/hardware/interface/arx_r5.py`, `teleop_details.md`.
- Relevant behavior: controller-grip clutch captures controller and current
  end-effector references, computes relative translation/rotation, resets
  references on release, applies a headset/world rotation, and maps analog
  trigger values to gripper positions.
- Design adopted: relative device and robot references, release-time reference
  reset, configurable signed axis mapping, and capability-driven gripper input.
- Design rejected: device, IK, cameras, logging, visualization, and hardware
  setup in one inheritance tree; Placo as the mandatory control path; automatic
  home motion during setup; PICO and dual-arm assumptions.
- Code copied or modified: None.
- Attribution required: None for Phase 0. Preserve the MIT notice if code is
  later copied.
- Relevant tests: No teleoperation contract suite was found; only a data-log
  analysis script is named as a test.

## pantor/ruckig

- Repository: https://github.com/pantor/ruckig
- Commit SHA: `a8db97a4e9c55e5160a3855f739fa3b270df8e4c`
- License: MIT, copyright 2021 Lars Berscheid.
- Files inspected: `README.md`, `examples/01_position.py`,
  `examples/05_velocity.py`, `examples/06_stop.cpp`,
  `examples/15_tracking_offline.py`, `include/ruckig/ruckig.hpp`,
  `include/ruckig/input_parameter.hpp`, `include/ruckig/output_parameter.hpp`,
  `src/ruckig/brake.cpp`.
- Relevant behavior: online `update()` produces one cycle and
  `output.pass_to_input()` advances current position, velocity, and acceleration;
  position and velocity interfaces have explicit limits; a stop changes to a
  zero-velocity target. The predictive tracking interface is Pro-only.
- Design adopted: use the mature library in a future controller, update targets
  online, carry the full generated state forward, validate inputs, and use a
  limited velocity target for smooth braking.
- Design rejected: custom S-curve implementation, full trajectories per input
  packet, cloud waypoint calculation in the control path, and relying on the
  Pro tracking API for the first implementation.
- Code copied or modified: None.
- Attribution required: None for Phase 0. Preserve MIT notice when distributing
  Ruckig or copied code.
- Relevant tests: `test/test_target.cpp` covers online update, input validation,
  dynamic DoFs, velocity mode, zero limits, and randomized trajectories.

## stack-of-tasks/pinocchio

- Repository: https://github.com/stack-of-tasks/pinocchio
- Commit SHA: `f101785266e5367969c22103e2b5a3b59e5ace27`
- License: BSD 2-Clause, copyright CNRS and INRIA.
- Files inspected: `doc/b-examples/d-inverse-kinematics.md`,
  `doc/a-features/f-kinematic.md`, `examples/inverse-kinematics.py`,
  `examples/overview-urdf.py`, `include/pinocchio/algorithm/kinematics.hpp`,
  `include/pinocchio/algorithm/frames.hpp`,
  `include/pinocchio/algorithm/jacobian.hpp`.
- Relevant behavior: build a model from URDF, create reusable Data, run forward
  kinematics and frame placement updates, compute frame Jacobians in explicit
  reference frames, use manifold-aware integration, and damp IK near
  singularities.
- Design adopted: future backend/controller use for FK, frame pose, analytic
  Jacobian and singularity metrics, with model/data allocated outside the servo
  loop and explicit `LOCAL_WORLD_ALIGNED` conventions.
- Design rejected: importing the full dynamics/collision ecosystem in Phase 0,
  fixed-damping tutorial IK without robot-specific limits, and allocation-heavy
  model construction in a live cycle.
- Code copied or modified: None.
- Attribution required: None for Phase 0. BSD notice must be retained if code is
  copied or binary/source redistribution requires it.
- Relevant tests: `unittest/kinematics.cpp` covers FK; `unittest/frames.cpp`
  covers placements and frame Jacobians in local, world, and
  local-world-aligned frames.

## Secondary references

The following are recorded for later targeted review and were not required or
inspected in Phase 0: `real-stanford/umi-on-legs`, the XRoboToolkit C++ sample,
MoveIt 2 Servo, One Euro Filter, TeleVision, and Unitree `xr_teleoperate`.
Compliance references (`frankapy`, `libfranka`, and `franky`) are also deferred.
Franka torque-loop behavior must never be assumed to exist on ARX5.
