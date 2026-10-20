# robot-teleop

`robot-teleop` is a low-latency teleoperation framework whose input devices,
transport, canonical control logic, policies, controllers, and robot backends
are independently replaceable.

Phase 1 is complete. The Phase 2 simulator milestone is also implemented: the
repository now includes the Swift/SwiftUI iOS client, ARKit and simulated pose
providers, strict cross-language codecs, latest-only Network.framework UDP,
clock sync, session controls, and an operational controller UI. Direct
iPhone-to-Ubuntu Tailscale UDP, primary controls, background release, and
tracking/process-reset recovery pass. A three-minute iPhone trace remained in
the system `Nominal` thermal state and the latest direct UDP sample observed an
approximately 60 Hz fresh-Pose stream. The physical iPhone selected
`60 Hz 1280x720` and measured `60.0 FPS`; Phase 2 remains open for the
operator's thermal/battery impression and final physical ergonomics. The Phase 3
two-sided E-stop reset is implemented and validated against MockRobot. The
post-fix connected iPhone-to-Mock smoke sample passed, so Phase 4 static Ghost
review has started while longer active-control soak testing is deferred to the
pre-hardware gate.

## Safety status

- All default profiles and CI-equivalent commands remain hardware-free.
- The registered ARX5 backend fails closed until `--enable-hardware`, an explicit
  interface, a measured-joint FK provider, and the operator-only gates are
  supplied; its active Cartesian command path remains disabled.
- The Pose simulator defaults to Clutch released. `--clutch` is explicit and
  can only move the in-memory MockRobot in the current repository.
- Software E-stop behavior is not a substitute for a physical emergency stop.
- Phase 1 UDP is unauthenticated and must stay on a trusted, isolated network.
- The iOS app never auto-connects; its default endpoint is editable and no
  hardware backend exists in this repository.

## Quick start

```bash
uv sync --group dev
uv run robot-teleop run \
  --config configs/profiles/mock_mock.yaml \
  --duration 1.0
uv run pytest
```

The run command prints one JSON summary after shutdown. Receiver, Servo, and
metrics workers never print or write files in their hot paths.

## ARKit-format UDP to Mock

Run the Ubuntu-side dry-run receiver:

```bash
uv run robot-teleop run \
  --config configs/profiles/arkit_mock.yaml \
  --duration 5
```

In another terminal, send safe Clutch-released test poses:

```bash
uv run python tools/pose_simulator.py \
  --host 127.0.0.1 \
  --port 5005 \
  --duration 2
```

Useful finite diagnostics:

```bash
uv run python tools/pose_monitor.py --duration 5
uv run python tools/udp_loopback.py --count 100
uv run python tools/network_test.py \
  --duration 2 \
  --rate-hz 500 \
  --loss-rate 0.1 \
  --reorder-rate 0.1 \
  --jitter-ms 2
```

Every tool emits one machine-readable JSON report and exits. None selects a
real robot.

## Phase 3 Mock validation

Run the finite, deterministic ARKit-format UDP validation against MockRobot:

```bash
make phase3-mock-validation
```

The report covers signed-axis position mapping, orientation mapping, Clutch and
Recenter continuity, tracking and session recovery, malformed/jump/sequence
rejection, latest-only bursts, explicit two-sided E-stop reset, and staged
Watchdog stopping. It binds an ephemeral loopback UDP port and always reports
`"hardware_enabled": false`.

For a manual iPhone-to-Mock session, run the Ubuntu-side local console:

```bash
make phase3-mock-session
```

It accepts `status`, `report`, `reset`, and `quit` only. `report` emits a
bounded runtime snapshot with Servo frequency, jitter, packet age, protocol
events, and estimated sequence loss; read RTT from the iOS app because only it
has both Ping/Pong endpoints. `reset` is a local operator action: it succeeds
only after a matching iPhone reset request plus released, normal-tracking Pose
has made the Ubuntu E-stop reset gate ready. The console cannot select or
initialize a real robot.

On the current Ubuntu host, `uv` is installed outside the non-interactive PATH;
run `/home/r2/.local/bin/uv run python tools/phase3_mock_session.py` until the
operator elects to configure that shell PATH.

## Phase 4 Ghost review

Generate the hardware-free static Ghost mapping review:

```bash
make phase4-ghost-review
```

Open `build/phase4_ghost_review.html` locally. The page shows the active
ARKit-to-Mock profile mapping, translation projections, orientation axes, a
canonical-pipeline target/TCP trajectory, workspace bounds, and safety rejects.
It does not start networking, initialize CAN, or affect Servo timing.

Run the separate offline X5 model preflight with:

```bash
make phase4-kinematics-check
```

This installs the optional `kinematics` extra when needed, loads the pinned X5
URDF through Pinocchio, and writes `build/phase4_kinematics_check.json`. It
checks a deterministic relative Clutch/Recenter/rotation path for FK, bounded
seed-continuous IK, and Jacobian diagnostics. It has no ARX5 SDK, CAN, socket,
or hardware path. The copied model is source-attributed in
[`assets/robots/arx5/README.md`](assets/robots/arx5/README.md); see
[`docs/kinematics.md`](docs/kinematics.md) for scope and limitations.

## Phase 5 ARX5 foundation

The current Phase 5 branch adds an optional Ubuntu x86_64 ARX5 dependency set
and a fail-closed backend boundary. Install it only when working on the
offline foundation:

```bash
uv sync --extra arx5 --group dev
make phase5-check
make phase5-motion-preflight
```

On Ubuntu x86_64, this controller-free package inspection is also available:

```bash
make arx5-sdk-inspect
make phase5-sdk-model-check
make phase5-safety-contract
```

It has no CAN interface argument and does not construct an SDK controller; it
only prints static metadata packaged in the installed SDK wheel and confirms it
matches the attributed offline X5 model configuration.

`phase5-safety-contract` is also controller-free. It combines static SDK
position/velocity/torque fields with conservative local-reference candidates
for derivatives, Cartesian limits, and workspace, then reports the unresolved
physical evidence. Its JSON report always has `hardware_enabled: false` and
cannot authorize or send a robot command.

`make phase5-motion-preflight` is likewise hardware-free. It composes the
attributed X5 URDF, seeded Pinocchio IK, and latest-target online Ruckig at
200 Hz over a deterministic Clutch/Recenter pose path. Its derivative values
are an explicitly labelled offline benchmark profile, not verified physical-arm
limits.

The ARX5 Registry entry requires both `--enable-hardware` and an explicit
`--robot-interface` before it can even attempt SDK loading. The active Cartesian
IK/Ruckig command path is intentionally not enabled until the exact arm,
firmware, CAN interface, gripper calibration, limits, and stop behavior have
been confirmed. See [`docs/phase5_report.md`](docs/phase5_report.md).

### Ubuntu foreground ARX5 backend

If you SSH into Ubuntu directly and only need the phone-control backend, run:

```bash
cd /home/r2/work/projects/embodied/robot-teleop
./run_arx5_backend_foreground.sh
```

This stays in the SSH terminal foreground, prints logs there, and stops with
Ctrl-C. It does not pull code, reinstall packages, or leave a nohup/background
process. If an old backend is already running, the script stops that old
process first and then starts a fresh foreground backend. Defaults are CAN
`can0`, duration `86400`, phone endpoint `100.112.78.10:5005`, and
dashboard `http://100.112.78.10:8080/`.

### Mac control for the Ubuntu ARX5 backend

From the Mac repo root, use the helper script to control the Ubuntu-side ARX5
backend over SSH:

```bash
cd /Users/2-mo/Projects/robot-teleop

# Foreground mode: logs stay in this Mac terminal; Ctrl-C stops the backend.
./ubuntu_arx5_backend.sh run

# Background mode on Ubuntu.
./ubuntu_arx5_backend.sh start

# Stop, restart, inspect, or tail logs.
./ubuntu_arx5_backend.sh stop
./ubuntu_arx5_backend.sh restart
./ubuntu_arx5_backend.sh status
./ubuntu_arx5_backend.sh logs
```

The default SSH target is `r2@100.112.78.10`, with the remote repo at
`/home/r2/work/projects/embodied/robot-teleop`, CAN interface `can0`, and debug dashboard on port
`8080`. To use the robot's LAN address instead, override `REMOTE_HOST`:

```bash
REMOTE_HOST=192.168.5.8 ./ubuntu_arx5_backend.sh run
```

The dashboard then uses the same host and port, for example
`http://192.168.5.8:8080/`.

## iOS simulator

Open `apps/ios/RobotTeleop/RobotTeleop.xcodeproj` in Xcode, or run:

```bash
make ios-test
make ios-network-test
make ios-network-test-120
```

`ios-network-test` builds the app, starts a hardware-free Python protocol
receiver on `127.0.0.1:5005`, and verifies Pose, Ping/Pong, SettingsAck, and
disconnect through the iOS Simulator. `ios-network-test-120` uses a 120 Hz
simulator Pose source to exercise the same downstream UDP path. It is not an
ARKit sampling-rate assertion. The deployed app defaults to the direct Ubuntu
endpoint `100.112.78.10:5005`; the Mac is not part of that runtime path. See
[`apps/ios/README.md`](apps/ios/README.md) for simulator creation, Personal Team
signing, and the pending real-device checklist.

## Architecture

```text
Device -> byte Transport -> decoder/input adapter -> TeleopInput
       -> canonical Session/Mapping/Safety -> Policy -> Controller
       -> RobotBackend
```

Dependencies point toward `robot_teleop.core`. Core has no socket, filesystem,
ARKit, ROS, robot SDK, URDF-path, or framework dependency. See
[`docs/architecture.md`](docs/architecture.md),
[`docs/protocol.md`](docs/protocol.md),
[`docs/safety.md`](docs/safety.md), and
[`docs/extension_guide.md`](docs/extension_guide.md).

Phase reports:

- [`docs/phase0_report.md`](docs/phase0_report.md)
- [`docs/phase1_report.md`](docs/phase1_report.md)
- [`docs/phase2_report.md`](docs/phase2_report.md)
- [`docs/phase3_report.md`](docs/phase3_report.md)
- [`docs/phase5_report.md`](docs/phase5_report.md)
- [`docs/arx5_test_mapping.md`](docs/arx5_test_mapping.md)

Current phase status and the complete roadmap are tracked in
[`TODO.md`](TODO.md).

## Development

```bash
make sync
make check
make run-mock
make network-test
make ios-test
make ios-network-test
```

Quality gates run locally on Mac and Ubuntu. The UDP-backed Simulator UI checks
(`make ios-network-test` and `make ios-network-test-120`) remain explicit local
commands because they need a local Python monitor and have runner-dependent
Simulator scheduling. These checks never participate in the live
iPhone-to-Ubuntu control path.
