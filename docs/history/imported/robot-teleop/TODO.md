# Project Roadmap

Last updated: 2026-08-05

## Current status

```text
Completed:       Phase 0 and Phase 1; Phase 2 and Phase 3 are accepted for progression; Phase 4/5 software foundations are merged
Current gate:    Phase 5 ARX5 state-only readiness and reference alignment
In progress:     Phase 5 - no-motion backend evidence and state-only procedure
Next action:     Confirm physical limit candidates and prepare low-speed no-gripper control
Hardware state:  Disabled; ARX5 backend remains fail-closed without explicit hardware and interface flags
```

Phase 1 is complete. Phase 2 implementation, simulator verification, signing,
installation, direct Tailscale UDP, and primary physical controls pass. The
current capture/status build is installed and its 180-second direct UDP sample
observed an approximately 60 Hz fresh-Pose stream. The phone selected
`60 Hz 1280x720` and measured `60.0 FPS`; its 182.5-second Power Profiler trace
remained system-thermal `Nominal`. Phase 3 automated Mock validation and the
short post-fix connected-iPhone smoke sample are sufficient for software-only
progression. The tiring long connected active-control soak, final ergonomics,
and human coordinate/orientation approval are deferred to the pre-hardware
confidence gate instead of blocking no-motion backend work.
Ubuntu dry-run paths remain Mock/Replay input to MockRobot and
ARKit-format UDP to MockRobot. Evidence is in
[`docs/phase0_report.md`](docs/phase0_report.md),
[`docs/phase1_report.md`](docs/phase1_report.md), and
[`docs/phase2_report.md`](docs/phase2_report.md). Completed implementation plans
under `docs/plans/` are immutable historical records.

Connected-device deployment facts:

- Ubuntu host: `100.112.78.10`;
- Ubuntu username: `r2`;
- default UDP Pose/Ping endpoint: `5005`;
- server bind remains `0.0.0.0`; the host address is a deployment value and is
  not hard-coded into the protocol, core, or Ubuntu profile.

Confirmed Phase 2 client environment:

- target device: iPhone 16 Pro Max running iOS 26.5.2;
- Xcode 26.6 with the iOS 26.5 simulator runtime;
- deployment target: iOS 26.0;
- bundle identifier: `com.mo.robotteleop`;
- signing: Automatic Signing with a free Personal Team;
- development order: simulator first, then connected-device ARKit validation.

## How to maintain this file

- This is the single live status index; detailed designs and implementation
  plans belong under `docs/plans/`.
- Keep at most one phase marked **In progress**.
- Start a phase only after its scope is explicitly authorized.
- Mark work complete only when its tests, documentation, and exit criteria pass.
- Update this file in the same commit that changes a phase status.
- Link completed phases to their delivery report instead of copying test logs.
- Treat control thresholds and runtime structure as measurable design choices,
  not permanent rules.
- Never use roadmap status to bypass verified robot hard limits, explicit
  hardware enablement, or physical-safety gates.

## Optimization objective

Within the verified robot hard envelope, optimize aggressively for direct,
continuous control:

1. Minimize input-to-command P50/P95/P99 latency.
2. Minimize Servo jitter and deadline misses.
3. Avoid phase lag, target discontinuities, and visible velocity/acceleration
   steps.
4. Never execute obsolete targets or let latency grow through buffering.
5. Preserve a measured, predictable stop response.

The Phase 0 values, including 200 Hz and the 50/100/300 ms Watchdog stages, are
initial baselines rather than permanent targets. Filtering, deadzones, recovery
behavior, control rate, thread/process ownership, and Python/C++ boundaries may
change when benchmark and continuity tests show a better result.

## Phase overview

| Phase | Scope | Status | Exit evidence |
|---:|---|---|---|
| 0 | Architecture, references, canonical core, mocks, dry-run runtime | **Complete** | [`docs/phase0_report.md`](docs/phase0_report.md) |
| 1 | Binary protocol and Ubuntu UDP path | **Complete** | [`docs/phase1_report.md`](docs/phase1_report.md) |
| 2 | iOS ARKit application | **Accepted for Phase 3 progression** | [`docs/phase2_report.md`](docs/phase2_report.md); comfort observations deferred |
| 3 | iPhone-to-Mock end-to-end validation | **Accepted for Phase 4 progression; long active soak deferred** | [`docs/phase3_report.md`](docs/phase3_report.md) |
| 4 | Simulation and ghost visualization | **Software foundation complete; human mapping approval pending** | Static Ghost, offline FK/IK/Ruckig checks |
| 5 | ARX5 backend, IK, Ruckig, and robot-local safety | **In progress - active path disabled** | Hardware-disabled backend contract report |
| 6 | Low-speed physical validation | **Not started - operator only** | Signed manual checklist and test report |
| 7 | Advanced performance optimization | **Not started - conditional** | Before/after latency and jitter evidence |
| 8 | Replacement-component architecture proof | **Not started** | Extended integration matrix |

## Next actions

- [x] Obtain explicit authorization to begin Phase 1.
- [x] Write and approve a Phase 1 design and implementation plan.
- [x] Resolve the Phase 1 protocol decisions listed below.
- [x] Keep all robot and CAN paths unavailable during Phase 1.
- [x] Publish the Phase 1 delivery report and stop before Phase 2.
- [x] Obtain explicit authorization before starting Phase 2.
- [x] Confirm target device, OS, Xcode, signing, bundle identifier, and endpoint.
- [x] Approve the simulator-first Phase 2 design and implementation plan.
- [x] Complete simulator implementation and automated verification.
- [x] Connect, sign, install, and run the app on the target iPhone.
- [x] Install the capture/status update and record hardware-free UDP and system
  thermal samples on the target iPhone.
- [x] Record on-screen ARKit source telemetry: `60 Hz 1280x720`, measured
  `60.0 FPS`; current direct UDP cadence is approximately 60 Hz.
- [ ] Record the operator's thermal/battery impression and final physical
  ergonomics.
- [x] Add a Git remote and push the feature branch.
- [x] Disable GitHub Actions for this repository. Quality gates now run locally
  on Mac/Ubuntu with `make check`, `make phase5-check`, and explicit iOS
  commands instead of consuming hosted Actions minutes.

Phase 1 protocol decisions to resolve before implementation:

- [x] Assign stable `source_type` values and ownership of the registry.
- [x] Define `UInt32` sequence wrap behavior.
- [x] Define ping, pong, and clock-offset message layouts and versioning.
- [x] Set the raw quaternion norm tolerance before normalization.
- [x] Define allowed capability/validity-flag combinations.
- [x] Define session replay and old-session rejection behavior.

## Phase 0 - Architecture and foundation

Status: **Complete** at commit `1e9ca91`.

- [x] Initialize the independent `robot-teleop` repository on `main`.
- [x] Configure Python 3.12, uv, Ruff, strict mypy, pytest, and uv lockfile.
- [x] Add the GitHub Actions quality workflow.
- [x] Document the five layers, dependency direction, frames, protocol, safety,
  performance, hardware gate, and extension rules.
- [x] Inspect implementation files, revisions, and licenses for all six
  first-priority reference projects.
- [x] Record adopted designs, rejected designs, attribution, and code-copy status.
- [x] Define frozen canonical geometry, input, state, target, command, and
  capability models using `xyzw` quaternions.
- [x] Define Transport, InputSource, Policy, Compliance, Filter, Controller, and
  RobotBackend protocols.
- [x] Implement relative-pose mapping, signed axes, scaling, clutch, recenter,
  session continuity, state machine, and common safety checks.
- [x] Implement MockTransport, MockInputSource, ReplayInputSource,
  MockRobotBackend, RelativePosePolicy, and NoCompliancePolicy.
- [x] Implement explicit registries without dynamic discovery.
- [x] Implement latest-only storage and the Receiver, Servo, and Metrics workers.
- [x] Implement staged input watchdog, feedback-stale detection, deadline rebase,
  packet-age metrics, and safe shutdown.
- [x] Require release/re-press after tracking loss, network staleness, session
  change, and runtime reconnect.
- [x] Keep ARX5 unregistered and hardware execution unavailable.
- [x] Pass unit, contract, architecture, CLI, and Mock-to-Mock integration tests.
- [x] Publish the Phase 0 report and `arx5_test` function mapping.

Exit criteria:

- [x] Input, robot, and policy components can be replaced through their ports.
- [x] Core runs without ARX5, Xcode, ROS, socket, or filesystem dependencies.
- [x] Live input handoff is bounded and latest-only.
- [x] Dry-run is the only executable mode.
- [x] Local quality gates pass: Ruff, formatting, mypy, and 61 pytest tests.

## Phase 1 - Protocol and Ubuntu network path

Status: **Complete**.

- [x] Implement the strict 96-byte little-endian Python Pose packet
  encoder/decoder and shared Pose/Ping/Pong envelope.
- [x] Validate magic, major version, declared size, datagram length, enums,
  flags, finite fields, quaternion norm, and capability combinations.
- [x] Enforce session, sequence, timestamp, jump, and host-receipt-age rules.
- [x] Create canonical golden binary vectors and in-test corruption fixtures.
- [x] Implement `UdpTransport` with bounded buffers and clean shutdown.
- [x] Implement `ArkitPacketDecoder` separately from transport.
- [x] Implement `ArkitInputSource` that emits only canonical `TeleopInput`.
- [x] Integrate UDP reception with the existing latest-only runtime.
- [x] Implement low-rate ping/pong, RTT, clock-offset estimation, and outlier
  rejection without acknowledging every pose packet.
- [x] Add packet loss, duplicate, out-of-order, host-stale, session, and RTT
  metrics.
- [x] Add pose monitor, UDP loopback, network test, loss simulation, and latency
  report tools.
- [x] Test disconnect, restart, malformed traffic, burst traffic, and clean
  thread/resource shutdown.
- [x] Run ARKit-UDP-to-Mock locally without any robot dependency.
- [x] Publish a Phase 1 delivery report and stop before Phase 2.

Exit criteria:

- [x] Golden bytes round-trip exactly and all invalid packet classes are rejected.
- [x] Network bursts never create a growing queue or execute historical targets.
- [x] Clock-domain handling never reports raw client/host subtraction as latency.
- [x] Disconnect reaches hold, brake, and safe-stop stages as configured.
- [x] Core and all CI-equivalent local checks run without hardware SDKs.

## Phase 2 - iOS ARKit application

Status: **Complete for Phase 3 progression; operator comfort observations deferred**.

- [x] Create the Swift/SwiftUI Xcode project under `apps/ios/RobotTeleop`.
- [x] Configure camera permission and `ARWorldTrackingConfiguration` only.
- [x] Capture one pose at most once per new `ARFrame` through
  `ARSessionDelegate`.
- [x] Map normal, limited, and unavailable tracking states into protocol fields.
- [x] Encode the shared fixed-length binary protocol using little-endian fields.
- [x] Implement and verify direct endpoint UDP with `Network.framework`.
- [x] Implement Connect, Disconnect, hold-to-control, Recenter, gripper slider,
  position/rotation scales, and E-stop request controls.
- [x] Display tracking, send FPS, sequence, RTT, session, network, and connection
  state.
- [x] Implement low-rate ping/pong and haptic feedback.
- [x] Run the shared golden vectors in Swift and Python.
- [x] Verify that no robot, IK, URDF, CAN, workspace, or robot-limit logic exists
  in the app.
- [x] Verify portrait and landscape layouts on an exact iPhone 16 Pro Max iOS
  26.5 simulator.
- [x] Verify Swift Simulator to Python Pose/Ping/Pong/SettingsAck UDP round trip.
- [x] Verify the same Swift-to-Python downstream path at a 120 Hz simulator Pose
  cadence; physical ARKit cadence remains a separate device check.
- [x] Add real capture-format selection, source-cadence telemetry, and bounded
  UI status publication without changing the Pose/UDP path.
- [x] Add hosted Swift formatting, target compilation, and deterministic unit
  checks. UDP-backed Simulator UI checks remain explicit local commands because
  they require a local monitor and runner-dependent scheduling.

Exit criteria:

- [x] iPhone sends directly to Ubuntu without a Mac runtime hop.
- [x] Old ARFrame poses are never resent by a high-frequency timer.
- [x] Tracking unavailable produces an invalid state that stops active control.
- [x] Swift and Python agree on every golden byte and `xyzw` field.

Connected-device checklist:

- [x] Select the Personal Team and install `com.mo.robotteleop` on the iPhone.
- [x] Accept Camera and Local Network permissions and confirm Developer Mode.
- [x] Confirm normal, limited, unavailable, interruption, and recovery tracking.
- [x] Confirm direct UDP to `100.112.78.10:5005` over Tailscale with no Mac hop.
- [x] Measure real ARFrame cadence, RTT, loss, and interarrival jitter.
- [x] Confirm background Clutch release and foreground recovery.
- [x] Confirm Clutch, Recenter, gripper, scales, E-stop latch, and haptics.
- [x] Confirm full process restart clears the phone-local E-stop latch.
- [x] Assign production E-stop reset to a two-sided Phase 3 workflow.
- [x] Complete a 120-second lifecycle run without backlog or stale replay.
- [x] Install the capture/status update and run the 180-second direct UDP sample
  plus a 182.5-second Power Profiler trace; the trace remained `Nominal`.
- [x] Verify the corrected `Disconnect` layout on the physical iPhone.
- [x] Record selected capture FPS/resolution and measured on-screen ARFrame
  cadence: `60 Hz 1280x720`, measured `60.0 FPS`; 120 Hz was not available.
- [x] Authorize Phase 3 after the accepted device-validation merge.
- [ ] Follow up with the operator's thermal/battery impression and final
  portrait/landscape ergonomics; this does not block Mock validation.

## Phase 3 - iPhone-to-Mock end-to-end validation

Status: **Accepted for Phase 4 progression - Mock-only, no hardware backend is available**.

- [x] Add deterministic ARKit-format UDP to Ubuntu to MockRobot validation.
- [x] Verify configured position signs, scale, handedness, and quaternion
  orientation against an independent expected Mock target.
- [x] Verify first clutch, release, reactivation, and recenter continuity in
  the automated Mock scenario.
- [x] Verify tracking limited/unavailable and recovery behavior in the automated
  Mock scenario.
- [x] Verify session change, retired-session rejection, latest-only bursts, and
  Watchdog network-loss behavior in the automated Mock scenario.
- [x] Verify the host can wait in stationary `DISCONNECTED` before the first
  iPhone input, while the Watchdog still safe-stops after a later input loss.
- [x] Specify and test two-sided E-stop reset request/acknowledgement without
  automatically clearing the Ubuntu latch.
- [x] Confirm the physical iPhone stays reset-pending until the local Mock
  console completes the matching E-stop reset acknowledgement.
- [x] Verify duplicate, out-of-order, malformed, and input-jump rejection.
- [x] Regress source-rate-aware Cartesian target limiting: a normal 50 Hz
  `0.4 m/s` input remains active, and a faster continuous input is bounded
  without queuing or latching a session fault.
- [x] Keep release-only startup baselines stationary without arming the
  Watchdog; verify active-clutch input loss still latches safe-stop.
- [x] Measure local loopback RTT, packet age, receiver rate, servo rate, jitter,
  loss counters, and Clutch-release-to-HOLD time.
- [x] Add host-side accepted-Pose inter-arrival metrics to separate source/network
  gaps from low-rate iOS Ping/Pong RTT.
- [x] Complete and record the local ten-minute bounded-memory Mock soak.
- [x] Retain a bounded full-session metric window for the ten-minute Phase 3
  soak and label report sample scope explicitly.
- [x] Re-run iPhone ARKit to UDP to Ubuntu to MockRobot using the operator
  console after the source-rate timing correction; the short post-fix sample
  had no Servo fault or Mock safe stop (full evidence is in the Phase 3 report).
- [ ] Measure a longer connected active-control sample, including
  Clutch-release-to-HOLD time, before any hardware backend work.

Exit criteria:

- [ ] Every coordinate direction is physically reviewed using Mock/Ghost only.
- [x] Automated interruption and recovery paths do not create a target jump.
- [x] No command backlog or latency growth appeared during the local soak or the
  short post-fix connected-iPhone smoke sample.
- [ ] Repeat a longer connected active-control sample before hardware work.

## Phase 4 - Simulation and ghost visualization

Status: **Software foundation complete - static Ghost plus offline X5 preflight,
hardware disabled; human mapping approval pending**.

- [x] Select a lightweight visualization path: static HTML/SVG Ghost review first.
- [x] Generate mapped target ghost cases, workspace bounds, scales, and axis signs.
- [x] Generate a deterministic pipeline-backed target/TCP trajectory with Clutch,
  Recenter, release, reactivation, and workspace-reject samples.
- [x] Display target Ghost, Mock TCP, workspace bounds, and safety rejects outside
  the Servo thread.
- [x] Add model loading outside `core/` using an attributed X5 URDF and an
  optional Pinocchio adapter.
- [x] Add offline FK/Jacobian and bounded seeded-IK branch-continuity checks for
  deterministic relative Clutch/Recenter/rotation targets.
- [x] Keep visualization and the model preflight outside the Servo thread and
  hardware disabled.
- [ ] Confirm input-to-robot axis and orientation mapping visually. This is
  deferred rather than blocking software-only progress, but remains mandatory
  before any hardware command is authorized.

Exit criteria:

- [ ] Mapping and orientation receive explicit human approval.
- [ ] Live targets stay inside the configured workspace and feasible IK region.
  The deterministic Phase 4 model path passes; live and hardware-specific
  validation remain pending.
- [x] Offline visualization/model tooling is structurally isolated from Runtime
  and cannot alter Servo cadence.

## Phase 5 - ARX5 backend and motion controller

Status: **In progress - SDK boundary, offline Ruckig foundation, and local
`arx5_test` evidence alignment; active path disabled**.

- [x] Re-review the selected ARX5 SDK revision and license before reuse. The
  pinned `arx5-interface==0.1.3` wheel is MIT-licensed and maps to the reviewed
  upstream SDK revision in `docs/references.md`.
- [x] Re-read the Ubuntu local reference checkout at
  `/home/r2/Desktop/arx5_test` commit
  `69a144f1cd7b6157a536428096b1f52e25d9200e` for configuration, lifecycle,
  watchdog, gripper, and stop-semantics evidence. No source was copied.
- [x] Add a controller-free static SDK inspector for packaged X5 model and
  controller metadata. It accepts no CAN interface and never constructs an SDK
  controller; its output is evidence about the package, not the physical arm.
- [x] Compare the installed SDK's static X5 joint envelope and frame names with
  the attributed offline model configuration in an isolated CI/tool check.
- [x] Build a hardware-disabled safety-contract report that intersects static
  SDK position/velocity/torque metadata with conservative local-reference
  candidates, marks the result unverified, and reports every physical blocker.
- [x] Add an explicit ARX5 state-only readiness tool. Default execution is a
  dry run; hardware access requires `--enable-hardware`, `--state-only`,
  `--robot-interface`, and `--confirm-operator-present`.
- [ ] Confirm arm model, joint count, firmware, CAN interface, units, frames,
  timestamps, command modes, and gripper behavior. The local reference provides
  likely defaults such as X5 and `can0`, but independent physical identity,
  firmware, units, frames, and command-mode evidence remain pending.
- [x] Re-run ARX5 state-only readiness with a five-sample window. The operator
  run passed with SDK timestamps `0.150835, 0.250345, 0.349863, 0.451669,
  0.550924` and advancing host timestamps; no framework command writes occurred.
  The vendor SDK still cleared motor-controller state, started background I/O,
  and entered damping on exit.
- [ ] Reconcile the observed gripper raw position `-0.0000834` with the local
  reference's `-3.24846` readout before normalizing or commanding the gripper.
  Current decision for first phone-to-arm test: leave gripper disabled.
- [x] Verify the SDK damping safe-stop primitive on the target arm. The operator
  confirmed presence, physical E-stop reachability, and cleared workspace; the
  gated run sent one `set_to_damping()` request, returned `safe_stop_passed=true`,
  and observed post-stop maximum joint velocity below the `1.0 rad/s` gate.
  The latest SSH-run evidence on `can0` observed
  `0.12088 rad/s` maximum post-stop joint velocity.
- [x] Implement an import-isolated `Arx5Backend` with explicit lifecycle. The
  constructor and Registry do not import the SDK; `connect` is gated by both
  the explicit hardware flag and a named interface.
- [x] Implement canonical state conversion and backend-boundary quaternion/order
  conversion helper for the evidenced SDK `XYZ + fixed-axis RPY` state shape.
  Runtime TCP feedback requires an injected measured-joint FK provider; gripper
  normalization remains pending calibration evidence.
- [x] Add an injectable robot-local ARX5 safety envelope for joint position,
  velocity, acceleration, jerk, torque, workspace, Cartesian velocity, and
  singularity validation. It is fake-tested at the SDK boundary and cannot
  enable the active command path.
- [ ] Verify and wire physical-arm joint, velocity, acceleration, jerk, torque,
  workspace, singularity, SDK, CAN, feedback, and gripper limits into the active
  command path. Static and local-reference values remain candidates only.
- [x] Compose Pinocchio FK, frame pose, Jacobian, bounded seeded IK, and generic
  latest-target Ruckig in a hardware-free deterministic preflight outside
  `core/`. Runtime composition remains disabled pending physical limits and
  state semantics.
- [x] Add a generic upstream online Ruckig wrapper; do not implement a custom
  S-curve. It is optional and tested independently of the SDK.
- [x] Carry Ruckig position, velocity, and acceleration state forward each
  cycle in the generic wrapper. Wiring it to ARX5 awaits verified IK and limits.
- [x] Implement an offline Cartesian position composition with smooth Ruckig
  braking, bounded seeded IK, target continuity rejection, and optional
  singularity-derived speed scaling. It is not wired to the SDK.
- [x] Latch malformed/stale feedback as a backend fault and attempt the SDK
  damping primitive on the active-boundary configuration; preserve a damping
  failure instead of claiming a successful stop. State-only explicitly disables
  this framework damping attempt.
- [x] Verify robot-specific SDK safe-stop acceptance on the target SDK/CAN/arm.
  Feedback-stale behavior is still covered only by fake/backend tests until a
  separate physical fault test is authorized.
- [x] Add backend and controller contract tests using mocks/fakes only.
- [x] Require both registered hardware support, explicit `--enable-hardware`,
  and an explicit `--robot-interface` before ARX5 composition.
- [x] Keep `hardware.enabled: false` in every default profile.

Exit criteria:

- [x] All automated tests run without a connected robot or ARX5 SDK in core
  tests. Optional algorithm/adapter tests use fake SDK objects.
- [x] No constructor, CI job, or default CLI command initializes CAN or moves;
  the optional CI job only installs wheels and runs fakes.
- [x] Hardware state-read and SDK damping stop semantics are documented from
  target-arm evidence; full physical limits remain pending.
- [x] State-only and safe-stop procedures are ready for and have passed operator
  execution. Next gate is physical limit candidate confirmation before low-speed
  no-gripper control.

## Phase 6 - Low-speed physical validation

Status: **Not started; manual operator phase only**.

- [ ] Operator confirms secure mounting, cleared workspace, and physical E-stop.
- [ ] Operator confirms the exact robot, firmware, SDK, CAN interface, and limits.
- [ ] Read state without enabling motion.
- [ ] Test backend safe stop before any commanded movement.
- [ ] Test network loss, tracking unavailable, and clutch release stops.
- [ ] Test one axis at minimum speed, then XYZ, then orientation, then gripper.
- [ ] Increase speed only after reviewing logs, limits, and stop behavior.
- [ ] Tune filtering from measured latency and jitter, never to hide faults.
- [ ] Produce a signed hardware and performance report.

Exit criteria:

- [ ] Every checklist item in `docs/hardware_checklist.md` is confirmed by the
  operator beside the arm.
- [ ] Physical stop behavior is measured and accepted at low speed.
- [ ] Codex and CI have not initiated robot motion.

## Phase 7 - Measured performance optimization

Status: **Not started; conditional on measurements**.

- [ ] Carry an explicit latency and smoothness budget forward from Phases 1-6;
  Phase 7 is for deeper optimization, not the first performance work.
- [ ] Profile receiver, mapping, filtering, IK, Ruckig, SDK calls, and Servo timing.
- [ ] Identify the measured source of P95/P99 latency or jitter.
- [ ] Optimize Python allocation, locking, scheduling, or SDK boundaries first.
- [ ] Isolate logging, visualization, and future model inference from Servo.
- [ ] Migrate only the demonstrated bottleneck to C++ if 200 Hz remains unstable.
- [ ] Repeat ten-minute and failure-path benchmarks after every optimization.

Exit criteria:

- [ ] Every optimization has before/after evidence and no weakened safety test.
- [ ] C++ is introduced only if Python/GIL/IK/SDK measurements justify it.

## Phase 8 - Extension validation

Status: **Not started**.

- [ ] Add at least one replacement component: SpaceMouse input, VR input,
  Cartesian velocity policy, or Mock UR backend.
- [ ] Run the applicable shared contract suite.
- [ ] Extend the integration matrix without copying Servo, watchdog, filtering,
  session, mapping, or metrics code.
- [ ] Document any interface gap and change the narrowest owning boundary only.

Exit criteria:

- [ ] The new component works without modifying unrelated adapters or core logic.
- [ ] Architecture tests demonstrate that common real-time behavior remains single
  ownership.

## Cross-cutting engineering policy

### Hard boundaries

These remain non-negotiable because violating them can cause unintended robot
motion or misrepresent physical protection:

- Real hardware is disabled by default and requires explicit operator enablement.
- E-stop remains latched until explicit reset; software E-stop is never described
  as a replacement for a physical emergency stop.
- A frontend or common policy cannot exceed verified RobotBackend hard limits.
- CI and Codex do not initiate robot motion.

### Performance invariants

These exist to improve responsiveness as much as safety:

- Live control remains latest-only; obsolete commands are discarded rather than
  queued.
- Blocking I/O, high-rate logging, video, network acknowledgements, and model
  inference stay off the Servo path unless measurements prove an equivalent or
  better deadline result.
- Every filter can be disabled for raw-latency measurement and must justify its
  phase lag with a measurable smoothness improvement.
- Input-to-command latency, jitter, packet age, target continuity, and stopping
  time are measured together; optimizing only average loop rate is insufficient.

### Reviewable choices

The following are intentionally not permanent constraints:

- 200 Hz control rate and the three Watchdog timeout values.
- Filter type, cutoff, beta, deadzone, and prediction strategy.
- Release/re-press behavior after brief tracking or network degradation.
- Python versus C++, thread versus process, and exact component placement.
- Common target-step, velocity, acceleration, and jerk limits below the backend
  hard envelope.

Change these through benchmarks, failure-path tests, and an ADR when the change
is architectural. Prefer the lowest measured latency and best continuity that
still passes the verified stop and backend-limit tests.

License checks, contract tests, and integration-matrix updates remain normal
engineering quality requirements rather than motion constraints.
