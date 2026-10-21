# Phase 1 Delivery Report

Date: 2026-08-03

Status: complete within the Phase 1 boundary. The repository can receive
ARKit-format UDP and drive only `MockRobotBackend`. It contains no iOS app, real
robot backend, CAN initialization, IK, Ruckig, or hardware motion path.

## Delivery summary

| Deliverable | Result |
|---|---|
| Common versioned binary envelope | Pose 96 B, Ping 36 B, Pong 52 B |
| Python codec | Strict encode/decode with stable rejection codes |
| Golden vectors | Three binaries plus cross-language JSON manifest |
| Sequence/session validation | UInt32 wrap, gaps, duplicates, order, retired sessions |
| Clock sync | Four-timestamp RTT/offset and bounded low-RTT estimator |
| UDP transport | IPv4, bounded drain, latest-only, peer reply, clean stop |
| ARKit adapter | Capability checked, canonical `TeleopInput`, Ping/Pong |
| Runtime integration | ARKit UDP to Receiver/Servo/MockRobot |
| Metrics | Protocol events, host age, jitter, loss, rejects, distributions |
| Diagnostics | Simulator, monitor, loopback, impairment test, latency report |
| Hardware state | Disabled and unavailable |

Implementation checkpoints:

- `304f9cb` - versioned codecs, sequence/session ordering, and clock estimator;
- `ec7c7d0` - checked-in protocol golden vectors;
- `6dc21d8` - latest-only UDP and ARKit input contracts;
- `9044e4c` - configuration, registry, metrics, CLI, and dry-run assembly;
- `83af143` - full ARKit-UDP-to-Mock session and Watchdog integration;
- `0fec086` - finite network diagnostic tools.

## Repository additions

```text
configs/profiles/arkit_mock.yaml
src/robot_teleop/
|-- inputs/arkit.py
|-- protocol/{packet,sequence,clock_sync,golden}.py
|-- transports/udp.py
`-- tools/{pose_simulator,pose_monitor,udp_loopback,network_test,latency_report}.py
tests/
|-- golden/{pose_v1,ping_v1,pong_v1}.bin
|-- contract/{test_arkit_input_contract,test_udp_transport_contract}.py
`-- integration/{test_arkit_udp_to_mock,test_tools}.py
tools/{pose_simulator,pose_monitor,udp_loopback,network_test,latency_report}.py
```

The root tool files are thin executable entry points. Testable implementations
live in the installed `robot_teleop.tools` package.

## Architecture and dependency direction

```text
UDP bytes
  -> UdpTransport
  -> generic packet codec
  -> ArkitPacketDecoder / ArkitInputSource
  -> canonical TeleopInput
  -> InputReceiver / LatestValue
  -> common Session, Mapping, Policy, Safety, Watchdog
  -> CartesianPositionController
  -> MockRobotBackend
```

`UdpTransport` has no Pose or robot semantics. The generic codec accepts stable
registered sources; each input adapter supplies expected source and capability
constraints. `core/` still imports no socket, protocol, ARKit, filesystem, SDK,
ROS, or robot dependency. ARKit-to-UDP-to-Mock is assembled only in the explicit
registry.

## Protocol v1.0

The approved common header is 36 bytes and dispatches by explicit packet type.
All fields are little-endian; quaternions are always `x, y, z, w`. The decoder
validates magic, exact major/minor version, declared and actual size, type,
source, zero reserved fields, known flags, all finite floats, raw quaternion
norm `[0.95, 1.05]`, tracking/flag consistency, capability claims, gripper
range, and nonzero session ID.

The Pose stream uses unsigned serial arithmetic. Delta zero is duplicate,
`[1, 2^31)` is forward progress including wrap, and the other half-range is old
or ambiguous. Forward gaps are accepted and counted. Recently replaced session
IDs are retained in a bounded retired set to reject delayed old-session traffic.

Golden Pose/Ping/Pong files match encoder bytes and decoder fields exactly. The
manifest stores UInt64 values as hexadecimal strings so Phase 2 does not lose
timestamp precision through JSON number conversion.

## Latency and latest-only behavior

UDP performs one blocking receive followed by a bounded nonblocking drain.
Queued older datagrams are counted as `udp.superseded` and never decoded or
published. The runtime then overwrites one `LatestValue`; neither layer has an
unbounded command queue.

During integration, a macOS-specific regression was found: `MSG_DONTWAIT` on a
socket with a configured timeout delayed an empty drain probe by roughly 100 ms.
The transport now enters an explicit nonblocking section for the bounded drain,
then restores the blocking timeout. A contract test with a 1 s main timeout
requires a ready single datagram to return within 250 ms.

Ping/Pong uses `t0/t1/t2/t3` to compute RTT and server-minus-client offset. Bad
intervals and samples above the configured RTT limit are rejected; a bounded
lowest-RTT subset supplies the median offset. Pose has no ACK. Raw Ubuntu minus
iPhone timestamp is never reported as one-way latency.

## Functional verification

The end-to-end integration test exercises:

- Clutch release, activation reference capture, relative mapped motion, and
  release HOLD;
- Ping/Pong on the Pose endpoint without per-Pose ACK;
- duplicate, out-of-order, gap, session change, and retired-session behavior;
- UDP burst disposal with the final sequence reaching MockRobot runtime;
- tracking unavailable and receipt-age HOLD, STOP, then backend safe-stop;
- clean exit of Receiver, Servo, metrics, socket, and MockRobot lifecycle.

Codec mutation tests cover wrong length, truncation, magic, major/minor version,
declared size, packet type, source, flags, header/payload reserved fields,
zero session, NaN, bad quaternion, tracking mismatch, Clutch inconsistency,
capability mismatch, and gripper range.

## Local quality result

Environment: Apple M3 Max, macOS 26.5.2, Python 3.12.12.

| Check | Result |
|---|---|
| `uv run ruff check .` | pass |
| `uv run ruff format --check .` | pass |
| `uv run mypy` | pass, 55 source files |
| `uv run pytest` | pass, 150 tests |
| ARKit-format UDP-to-Mock integration | pass |
| Mock CLI, 1 s | exit 0; no fault or safe-stop |
| ARKit UDP CLI, no sender, 50 ms | exit 0; clean HOLD and shutdown |

The GitHub Actions workflow runs the same locked checks on Ubuntu. No Git remote
is configured, so no hosted Actions run can truthfully be reported yet.

## Local performance baseline

These are one-machine regression samples, not real iPhone/Wi-Fi or real-time
guarantees:

| Measurement | Result |
|---|---:|
| Pose encode, `timeit`, best of 5 x 100,000 | 4.72 us |
| Pose decode, `timeit`, best of 5 x 100,000 | 13.6 us |
| 100 Ping/Pong loopback loss | 0 |
| Ping/Pong RTT median / P95 / P99 | 0.0982 / 0.1136 / 0.1713 ms |
| Ping/Pong RTT max | 0.2758 ms |
| 1000 Hz x 2 s attempted/sent | 2000 / 2000 |
| 1000 Hz decoded / superseded | 1999 / 1 |
| 1000 Hz final expected/observed sequence | 1999 / 1999 |
| Mock Servo, 1 s frequency | 199.90 Hz |
| Mock Servo P95 compute / deadline misses | 0.108 ms / 0 |

An impairment run at 500 Hz for 2 s used 10% configured loss, 10% reorder, up
to 2 ms jitter, and seed 7. It attempted 1000 packets, intentionally dropped 84,
sent 916, decoded 797, superseded 119 queued datagrams, and still ended at the
expected latest sequence 999. Counts show obsolete work was discarded rather
than accumulating latency.

## Safety and trust boundary

- Dry-run is the only registered path and `hardware_enabled` remains false.
- No source, config, CLI command, test, or tool imports or initializes ARX5/CAN.
- The simulator defaults to Clutch released; activation requires `--clutch`.
- E-stop remains latched in canonical state but is not a physical E-stop.
- Watchdog values remain tunable baselines, not permanent conservative limits.
- UDP v1 has no authentication or encryption and is for a trusted isolated LAN.

## Reference and license result

Phase 1 used the Phase 0 file-level reference review and copied no source code
from any reference repository. Implementation uses Python standard-library
`struct`, `socket`, threading, and monotonic clocks. Existing license findings
and attribution boundaries remain in [`references.md`](references.md).

Adopted in Phase 1:

- explicit fixed-size message dispatch and golden cross-language fixtures;
- latest-only kernel backlog disposal and one-value runtime handoff;
- serial-number wrap comparison and retired-session rejection;
- low-rate four-timestamp clock estimation with no Pose ACK;
- stable rejection/event codes and bounded metric samples;
- transport/decoder/input separation and explicit registry assembly.

Rejected in Phase 1:

- JSON, protobuf, per-Pose ACK, retransmission, or reliable historical queues;
- source-specific robot logic or decoding inside `UdpTransport`;
- raw cross-clock subtraction labeled as one-way latency;
- network authentication claims without an actual authenticated protocol;
- iOS, ARX5, CAN, IK, Ruckig, filtering, compliance, visualization, or hardware
  work beyond the authorized phase.

## Remaining risks and unimplemented work

- The Swift encoder/decoder and iPhone sender have not consumed the golden
  vectors; cross-language verification is Phase 2.
- Real Wi-Fi RTT, loss, burst patterns, device thermal behavior, and ARFrame
  cadence are unmeasured.
- Ubuntu cannot infer true source-to-receive one-way age from an unsynchronized
  Pose timestamp alone. It uses host receipt age and never overstates it.
- The two-second network tests prove bounded latest behavior, not ten-minute
  memory stability. The real iPhone-to-Mock ten-minute run is Phase 3.
- Python performance is measured without IK or a blocking robot SDK. The
  Python/C++ boundary remains evidence-driven in later phases.
- STOP is immediate in MockRobot. Smooth braking awaits online Ruckig and a
  verified robot-local controller.
- Filtering is still pass-through; it must be tuned only after raw latency and
  real tracking noise are measured.
- Hardware-specific joint, torque, singularity, CAN, feedback, and safe-stop
  capabilities remain unconfirmed exactly as listed in the Phase 0 report.

## Phase 2 plan

Phase 2 should create only the Swift/SwiftUI ARKit client:

1. Create the Xcode project and camera permission boundary.
2. Capture each new `ARFrame` at most once with `ARSessionDelegate`.
3. Encode the checked-in Pose/Ping vectors byte-for-byte in Swift.
4. Send UDP directly from iPhone to Ubuntu using `Network.framework`.
5. Implement Clutch, Recenter, gripper, E-stop request, scales, and status UI.
6. Estimate RTT/offset from Pong and reject bad samples without Pose ACK.
7. Run the same golden files in Python and Swift tests.

The first Ubuntu deployment target is known: host `100.112.78.10`, username
`r2`, with UDP port `5005` as the current profile default. The Ubuntu service
binds `0.0.0.0`; the iOS client will own the destination address. The host has
not been contacted during Phase 1.

Before Phase 2 implementation, user input is still useful for the Apple
development team/bundle identifier, minimum iOS version, and target iPhone
model. None of those choices blocks or changes this completed Phase 1 delivery.
