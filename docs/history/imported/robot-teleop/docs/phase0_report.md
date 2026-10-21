# Phase 0 Delivery Report

Date: 2026-08-03

Status: complete within the Phase 0 boundary. The repository contains no real
robot backend, performs no CAN operation, and cannot issue a hardware command.
Work stopped before Phase 1.

## Repository tree

```text
robot-teleop/
|-- .github/workflows/ci.yml
|-- apps/ios/README.md
|-- configs/
|   |-- inputs/{arkit,mock}.yaml
|   |-- policies/relative_pose.yaml
|   |-- profiles/{arkit_arx5,mock_mock}.yaml
|   `-- robots/{arx5,mock}.yaml
|-- docs/
|   |-- adr/
|   |-- plans/
|   |-- architecture.md
|   |-- arx5_test_mapping.md
|   |-- coordinate_frames.md
|   |-- extension_guide.md
|   |-- hardware_checklist.md
|   |-- performance.md
|   |-- protocol.md
|   |-- references.md
|   `-- safety.md
|-- src/robot_teleop/
|   |-- core/
|   |-- inputs/
|   |-- transports/
|   |-- policies/
|   |-- controllers/
|   |-- robots/
|   |-- runtime/
|   |-- protocol/
|   |-- config.py
|   |-- registry.py
|   `-- cli.py
|-- tests/{unit,contract,integration}/
|-- LICENSE
|-- Makefile
|-- README.md
|-- pyproject.toml
`-- uv.lock
```

## Architecture and dependency direction

The executable path is input adapter, transport/decoder, canonical core,
policy/controller, then robot backend. `core/` contains immutable canonical
models, geometry, clutch/session logic, mapping, state, common safety, and port
protocols. It uses only the Python standard library.

Dependencies point from CLI, registry, runtime, and adapters toward `core/`.
An AST architecture test rejects forbidden imports from the core. There is no
source-name branch in common behavior and no dynamic plugin discovery.

Each live concern has one owner: `LatestValue` for overwrite-only handoff,
`TeleopSession` for clutch/recenter continuity, `TeleopStateMachine` for state
and latches, `Watchdog` for receipt freshness, and `ServoLoop` for cadence and
feedback progress.

## Phase 0 deliverables

| Deliverable | Result |
|---|---|
| Repository and Python 3.12/uv setup | Complete |
| Ruff, strict mypy, pytest, GitHub Actions workflow | Complete |
| Architecture, extension, protocol, frames, safety, performance docs | Complete |
| Six first-priority reference implementations and licenses reviewed | Complete |
| Frozen canonical models and capability records | Complete |
| Structural port protocols | Complete |
| Explicit registries | Complete |
| Mock transport, mock/replay input, mock robot | Complete |
| Relative pose and no-compliance policies | Complete |
| Latest-only receiver and three-thread runtime | Complete |
| Basic state machine and staged watchdog | Complete |
| Contract, unit, architecture, and Mock-to-Mock tests | Complete |
| Binary protocol definition | Complete; codec and golden bytes are Phase 1 |
| iOS application | Not started by design; Phase 2 |
| ARX5/CAN/IK/Ruckig | Not started by design; Phase 5 |

## Core models and interfaces

Canonical models are `Vector3`, `Quaternion`, `Pose`, `Twist`, `Wrench`,
`TeleopInput`, `RobotState`, `RobotTarget`, and `RobotCommand`. Quaternions are
always normalized `x, y, z, w`. Inputs, targets, state, and configuration reject
invalid or non-finite values at their construction boundaries.

Ports are `Transport`, `TeleopInputSource`, `ControlPolicy`,
`CompliancePolicy`, `TargetFilter`, `Controller`, and `RobotBackend`.
`InputCapabilities` and `RobotCapabilities` drive feature selection.

The Phase 0 registry contains Mock and Replay inputs, Mock transport,
RelativePose policy, CartesianPosition controller, and Mock robot only. ARKit
and ARX5 configuration examples are non-executable placeholders. ARX5 is not
registered, and a non-Mock robot also requires the future explicit hardware
flag.

## Reference and license review

| Project | Revision | License finding |
|---|---|---|
| `2-mo/arx5_test` | `0dbffd38bd40` | no repository-level license found |
| `real-stanford/arx5-sdk` | `8ccbf0aa8d42` | MIT |
| `yihuai-gao/iPhoneVIO` | `adabceaf02f0` | no repository-level license found |
| XRoboToolkit Python sample | `79e5cb8a56e3` | MIT |
| Ruckig | `a8db97a4e9c5` | MIT |
| Pinocchio | `f101785266e5` | BSD 2-Clause |

No source code was copied or modified. The full file-level review and
attribution notes are in `references.md`.

Adopted designs include latest-only generation, fixed-period servo with
deadline rebase, stale-feedback detection, staged watchdog, relative clutch
references, signed axis mapping, explicit capabilities, backend-local hard
limits, and mature future Ruckig/Pinocchio boundaries.

Rejected designs include direct world-pose mapping, per-packet trajectories,
unbounded queues, source-name branching, automatic hardware initialization,
sample-script architecture, web/camera/model dependencies in the control path,
custom S-curves, network torque commands, and force/compliance control in the
first release.

## Mock-to-Mock operation

```bash
uv sync --group dev
uv run robot-teleop run \
  --config configs/profiles/mock_mock.yaml \
  --duration 1.0
```

The command starts a mock source, latest-only receiver, 200 Hz dry-run servo,
metrics worker, and Mock robot. It emits one JSON report after all workers join.
It never selects a real backend.

## Verification result

Local quality gates on macOS with Python 3.12.12:

| Check | Result |
|---|---|
| `ruff check .` | pass |
| `ruff format --check .` | pass |
| `mypy` | pass, 42 source files |
| `pytest` | pass, 61 tests in 0.34 s |
| one-second Mock-to-Mock CLI | exit 0, no receiver/servo fault |

The observed CLI sample ran 205 cycles at 199.81 Hz with zero deadline misses,
zero rebases, and zero safe stops. This one-second sample validates lifecycle
and instrumentation only; it is not a real-time guarantee or the required
future ten-minute stability test.

The GitHub Actions workflow runs the same locked dependency, lint, format,
type, and test gates on Ubuntu. No Git remote exists yet, so a hosted Actions
run has not been triggered. The equivalent local commands pass.

## Risks and deferred work

- Python scheduling under IK and robot SDK load is unmeasured.
- The binary protocol has a fixed 92-byte layout but no encoder, decoder,
  golden vectors, UDP transport, or clock synchronization yet.
- Phase 0 filtering is an explicit pass-through implementation; One Euro and
  quaternion adaptive filtering remain later measured work.
- Mock STOP is immediate. Smooth zero-velocity braking depends on future
  online Ruckig and verified backend behavior.
- A one-second local run does not cover packet loss, disconnects, long-duration
  memory behavior, or clutch-release braking time.
- Common workspace checks cannot substitute for verified ARX5 joint, torque,
  singularity, SDK, CAN, and mechanical limits.
- Repository-less reference licenses prohibit copying until permission or a
  license is established.

## Unconfirmed ARX5 capabilities

- Exact arm variant, joint count, firmware, and compatible SDK revision.
- Cartesian command semantics, update rate, units, frames, and quaternion order.
- Feedback timestamp clock, cadence, resolution, and stale-state behavior.
- Joint position, velocity, acceleration, jerk, torque, and gripper limits.
- CAN adapter/interface name, bitrate, exclusivity, and fault recovery behavior.
- Whether damping is the correct safe-stop primitive, and its latency and
  idempotence under SDK or CAN faults.
- Supported mode transitions and whether Cartesian control owns IK internally.
- External wrench availability and reliability; no compliance work may assume it.

Before any hardware phase, the operator must confirm the robot identity and
limits, mounting, cleared workspace, physical E-stop, CAN interface, and tested
safe-stop procedure. Hardware tests must be run locally by the operator beside
the arm; neither CI nor Codex may initiate motion.

## Phase 1 plan

1. Implement strict Python encoding and decoding for the documented 92-byte
   little-endian packet.
2. Add shared golden binary vectors and corruption/version/finite-value tests.
3. Implement UDP transport, ARKit decoder/source, and latest-only receiver
   integration without robot dependencies.
4. Add sequence, session, age, capability, jump, loss, and out-of-order metrics.
5. Implement low-rate ping/pong clock-offset estimation and RTT outlier filtering.
6. Add pose monitor, UDP loopback, loss simulation, disconnect tests, and a
   latency report.

Phase 1 remains unimplemented in this delivery.
