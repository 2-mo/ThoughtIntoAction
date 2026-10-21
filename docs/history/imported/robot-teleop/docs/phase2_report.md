# Phase 2 Delivery Report

Date: 2026-08-03, updated 2026-08-04

Status: simulator implementation complete and connected-device validation in
progress. Direct iPhone-to-Ubuntu UDP and the primary controls pass, while the
remaining lifecycle checks are listed below. No robot, ARX5 SDK, CAN interface,
IK, Ruckig, or hardware command path was added or run.

## Delivery summary

| Deliverable | Result |
|---|---|
| Xcode application | Swift 6, SwiftUI, iOS 26.0 target, bundle `com.mo.robotteleop` |
| Pose providers | Simulated at 60 Hz; ARKit one emission per new `ARFrame` |
| Protocol | Exact Pose, Ping, Pong, Settings, and SettingsAck golden vectors |
| UDP | Network.framework, one Pose in flight plus one replaceable pending Pose |
| Clock exchange | Low-rate four-timestamp Ping/Pong with bounded RTT filtering |
| Controls | Connect, Clutch, Recenter, gripper, scales, latched E-stop request |
| Status | Tracking, send FPS, sequence, RTT, session, network, settings state |
| Lifecycle | Release on gesture end/inactive; released final Pose on disconnect |
| UI | Operational portrait and landscape layouts with stable metric dimensions |
| Python diagnostic | Pose monitor acknowledges settings and reports bounded canonical control observations |
| Hardware state | Disabled and unavailable |

Implementation checkpoints:

- `6632998` - approved Phase 2 simulator-first design;
- `afb9f42` - session-scoped mapping settings control plane;
- `88c6c2c` - five shared golden vectors;
- `8faf2d5` - simulator-safe Xcode app and ARKit provider boundary;
- `2386973` - strict Swift codec and clock estimator;
- `12215e7` - latest-only UDP and session coordinator;
- `fd7f243` - immediate Clutch-release and E-stop packet edges;
- `0ffa180` - operational controller UI and responsive UI tests;
- `9dd1be7` - live Simulator-to-Python protocol round-trip test;
- `5ca44b4` - bounded mixed-protocol receive batches with no Pose/control loss;
- `ea5b919` - connected-device signing and Xcode workspace hygiene;
- `c7ef2f1` - input-independent control observations in `pose_monitor`;
- `454d9cc` - separate physical edges from controls active at session start.

## Capture cadence and UI efficiency follow-up

The first device run was functionally correct but the operator observed
noticeable phone warming and a clipped `Disconnect` label. The iOS client now:

- selects the highest device-provided ARKit camera FPS, then the lowest pixel
  count among formats at that FPS;
- sends only fresh `ARFrame` poses, never repeated poses to simulate 120 Hz;
- records selected resolution/FPS and a measured ARFrame cadence for the
  physical result;
- disables unused ARKit light estimation, HDR, environment-texture, audio, and
  frame-semantics work;
- publishes ordinary SwiftUI status at most 10 Hz while preserving immediate
  tracking, networking, RTT, settings, Clutch-release, and E-stop updates;
- gives `Disconnect` a stable 120-point width.

The 200 Hz Ubuntu Servo baseline is unchanged. If the iPhone supplies genuine
120 Hz ARFrames, it receives newer latest-only targets at that cadence. If it
only supplies 60 Hz ARFrames, it remains a measured 60 Hz source; ProMotion
display refresh is not treated as pose data.

New XCTest coverage proves format ranking, disabled configuration features,
status coalescing without Pose loss, immediate tracking publication, and the
minimum Disconnect width. The connected simulator screenshot was inspected and
the label, endpoint fields, metric cells, tuning controls, and Hold surface fit
without overlap. A new local Simulator-to-Python run received 61 fresh Pose
packets with 16.643 ms median and 17.492 ms P95 interarrival, one Ping/Pong,
and one Settings/SettingsAck exchange; hardware remained disabled.

The same hardware-free Swift-to-Python path also ran with an explicitly
configured 120 Hz simulator Pose source. It received 121 fresh Pose packets
with 8.297 ms median, 9.441 ms P95, and 14.737 ms maximum interarrival, plus one
Ping/Pong and one Settings/SettingsAck exchange. This demonstrates that the
downstream sender, UDP receiver, and decoder accept current targets at that
cadence without a queue. It deliberately does not establish that the physical
iPhone's ARKit session supplies 120 Hz ARFrames.

## 2026-08-04 physical capture and transport sample

Commit `bfa8af8` was signed, installed, and launched on the wired iPhone 16 Pro
Max running iOS 26.5.2. A hardware-free monitor on Ubuntu received the direct
Tailscale UDP stream for 180 seconds. It received 10,635 Pose datagrams and
observed 10,520 latest snapshots at 58.444 Hz. Interarrival timing was:

| Metric | Result |
|---|---:|
| Median | 16.678 ms |
| P90 | 19.532 ms |
| P95 | 21.320 ms |
| P99 | 28.418 ms |

The monitor completed 176 Ping/Pong exchanges and three Settings/SettingsAck
exchanges. It reported no `sequence.loss` or sequence-rejection event. The 115
`udp.superseded` events are expected latest-value replacements in the receiver,
not packet-loss evidence. Three observed sessions and the 1.869 s maximum gap
include the deliberate Disconnect/Connect transition, so that maximum is not a
steady-state cadence result.

This physical sample establishes an approximately 60 Hz end-to-end fresh-Pose
stream, not a 120 Hz result. The operator confirmed the updated App reports
`ARKit capture: 60 Hz 1280x720, measured 60.0 FPS`, with `Send FPS` at 60 Hz,
RTT near 8 ms, and a fully visible `Disconnect` label. Because the client sends
at most one Pose per `ARFrame`, the source and endpoint measurements agree: on
this iPhone/ARKit configuration, ProMotion display refresh does not provide a
120 Hz pose stream.

A separate 182.513-second Power Profiler trace attached to `RobotTeleop` on the
same iPhone. The device remained in the `Nominal` thermal state for the full
trace. Its first monitor setup was still resolving Ubuntu development
dependencies, so the trace is valid for foreground ARKit capture and outbound
send work but is not used to make a round-trip network claim. Operator thermal
comfort and the updated UI layout remain human observations.

## Architecture and dependency direction

```text
ARSessionDelegate / SimulatedPoseProvider
                  -> PoseSample
                  -> TeleopCoordinator
                  -> PacketCodec
                  -> UDPClient
                  -> Ubuntu ArkitInputSource
                  -> TeleopInput
```

The iOS app owns device capture, byte encoding, connection UI, controls, and
client metrics only. It contains no robot joint model, robot frame mapping, IK,
trajectory generation, workspace envelope, CAN, or robot SDK dependency.
Position and rotation controls send generic session-scoped settings; the common
Ubuntu relative-pose policy applies them only on the next reset/recenter so an
active target cannot jump.

`SimulatedPoseProvider` and `ARKitPoseProvider` implement the same boundary.
Changing the provider does not change the codec, transport, controls, Ubuntu
input adapter, policy, Servo loop, or MockRobot. Swift never imports Python and
Python core never imports Swift, ARKit, Network.framework, or Xcode resources.

## Protocol and timing behavior

The five packet layouts are fixed-size, little-endian, versioned, and checked
against the same repository files in both languages:

| Packet | Size |
|---|---:|
| Pose | 96 bytes |
| Ping | 36 bytes |
| Pong | 52 bytes |
| Settings | 44 bytes |
| SettingsAck | 44 bytes |

Quaternions are `x, y, z, w`. Swift rejects malformed size, magic, version,
reserved fields, non-finite values, and invalid quaternion norms. Pong and
SettingsAck must match the active session, request sequence, timestamp, and
settings values before they update UI state.

Pose sends are best-effort and never acknowledged. The client sender bounds
work to one in-flight Pose and one replaceable pending Pose. Clutch release and
E-stop request enqueue an immediate newest-state Pose rather than waiting for a
future AR frame. Reconnect generates a new nonzero session and forces its first
Pose to Clutch released. Settings retries until acknowledged; Ping abandons a
lost sample before issuing a later request.

## Simulator verification

The exact local visual destination was an iPhone 16 Pro Max on iOS 26.5. Both
portrait and landscape screenshots were inspected after UI tests. Content was
nonblank, all primary controls were visible and hittable, slider labels fit,
metric cells remained stable, and no overlap or clipping was observed.

The live network test launched the app against a Python `pose_monitor` on
`127.0.0.1:5005`. One observed session produced:

| Measurement | Result |
|---|---:|
| Accepted Pose packets | 61 |
| Pose interarrival mean / median | 16.662 / 16.670 ms |
| Pose interarrival P95 / max | 17.017 / 22.030 ms |
| Effective source cadence from mean interval | about 60.0 Hz |
| Ping received / Pong sent | 1 / 1 |
| Settings accepted / SettingsAck sent | 1 / 1 |
| Applied position / rotation scale | 0.7 / 0.8 (Float32) |
| Tracking | normal |
| Superseded datagrams / Pose sequence loss | 0 / 0 |
| Robot or hardware use | none |

An earlier live run exposed control datagrams being coalesced with Pose. The
final implementation uses the bounded protocol batch from ADR-0008: all controls
are processed, only the newest Pose is published, and no historical target queue
is created. The final run had no control loss, Pose sequence loss, or periodic
33 ms gap. Longer real-network stress and ten-minute stability remain Phase 3
measurements.

## Connected iPhone validation

The signed app was installed and opened on an iPhone 16 Pro Max running iOS
26.5.2 with Developer Mode enabled. The iPhone sent directly over Tailscale to
the Ubuntu host at `100.112.78.10:5005`; the Mac was not a runtime hop. Tailscale
reported a direct path, and host-to-phone control pings were about 5-6 ms.

A 60-second foreground sample produced:

| Measurement | Result |
|---|---:|
| Accepted Pose packets | 3,595 |
| Pose interarrival mean / median | 16.694 / 16.670 ms |
| Pose interarrival P95 / P99 / max | 19.335 / 24.888 / 35.955 ms |
| Effective source cadence | about 59.9 Hz |
| Pose sequence loss | 5, about 0.14% |
| Ping received / Pong sent | 59 / 59 |
| App RTT display | about 10 ms |
| Backlog or latency growth | none observed |
| Robot or hardware use | none |

The physical-control run observed two Clutch activation/release pairs, one
single-packet Recenter request, and gripper travel from `0.0` to `1.0` before
settling near `0.52`. Settings/SettingsAck completed with position scale `0.7`
and rotation scale `0.8`. The operator confirmed haptic feedback.

One E-stop press produced 5,191 subsequent `estop_requested=true` Pose packets,
no release edge, and remained requested across a reconnect/new session. A later
120-second observation received 7,182 more requested packets at 59.84 Hz with
no release. This confirms the E-stop latch and also confirms that ordinary
Disconnect/Connect and returning to the app do not reset it.

The current explicit phone-local test reset is full process termination and
relaunch, not Disconnect/Connect. The operator confirmed that this clears the
phone-local latch. A production in-app reset must be a two-sided workflow: the
phone requests a reset, Ubuntu verifies its latched safety state and operator
conditions, and the phone unlocks only after acknowledgement. A phone-only
unlock is deliberately not implemented because it could present an active UI
while the backend remains latched in `ESTOP`.

A subsequent 120-second lifecycle run observed 6,721 accepted Pose packets,
three sessions, two Clutch activation/release pairs, and no E-stop request. The
background transition created an expected 5.535-second receive gap; Clutch was
released before the gap, and the new session resumed at a 16.675 ms median
interarrival with no historical packet catch-up. Ping/Pong and SettingsAck
continued after foreground recovery.

A final permission-failure run observed four `unavailable` Pose packets followed
by 49 `limited` packets after Camera permission was restored. After three
session starts, one Ping/Pong and one Settings/SettingsAck round trip completed,
with no Clutch or E-stop request. The operator then confirmed that the UI
returned to `NORMAL`. Together with the earlier normal samples, this verifies
normal, limited, unavailable, interruption, and recovery behavior on the
physical device. Thermal behavior remains open.

## Automated checks

The checked-in workflow now has independent Ubuntu Python and `macos-26` iOS
jobs. The macOS job pins Xcode 26.6, formats Swift, and runs the app tests on an
iOS 26.5 simulator. GitHub's current standard runner catalog and image manifest
list that runner, Xcode, SDK, and simulator combination:

- <https://docs.github.com/en/actions/how-tos/write-workflows/choose-where-workflows-run/choose-the-runner-for-a-job>
- <https://github.com/actions/runner-images/blob/main/images/macos/macos-26-arm64-Readme.md>

Local quality results:

| Check | Result |
|---|---|
| `uv run ruff check .` | pass |
| `uv run ruff format --check .` | pass, 124 files |
| `uv run mypy` | pass, 57 source files |
| `uv run pytest` | pass, 166 tests |
| Swift strict format | pass |
| Full iPhone 16 Pro Max Simulator suite | 31 pass, 2 conditional network skips |
| Live Swift-to-Python UDP UI tests | 60 Hz and 120 Hz simulator sources pass |
| Generic iOS Simulator Release build | pass |
| Generic physical iOS Release build, unsigned | pass |
| `Info.plist` validation | pass |

The conditional network test skips when no Python listener is present so normal
Xcode test runs remain self-contained. `make ios-network-test` starts the
listener and requires both the XCTest and monitor's structured
`round_trip_complete` result to pass. The feature branch is pushed to the Git
remote. GitHub Actions run
[`30826664935`](https://github.com/2-mo/robot-teleop/actions/runs/30826664935)
completed successfully for commit `454d9cc`; the two preceding feature-branch
runs also completed successfully.

## Personal Team result

The signed-in free Apple account is sufficient for installing and testing the
app on the owner's phone. Apple currently limits Personal Team provisioning
profiles to seven days, after which the app must be rebuilt and reinstalled.
It does not permit App Store distribution. See Apple's account documentation:

<https://developer.apple.com/help/account/basics/about-your-developer-account>

Simulator tests do not require signing. Physical-device Automatic Signing with
the selected Personal Team, installation, launch, Camera access, Local Network
access, and direct UDP are now verified on the target phone.

## Remaining device gate

The remaining connected-device checks are:

- record the operator's thermal and battery impression after the current build;
- complete the final portrait/landscape ergonomics review on the physical
  iPhone.

The production two-sided E-stop reset is intentionally assigned to the Phase 3
iPhone-to-Mock state-machine workflow, where both request and acknowledgement
can be tested. Seven-day Personal Team reprovisioning remains an operational
constraint rather than a Phase 2 acceptance delay.

These items are tracked in `TODO.md`. Phase 3 is not started, and no physical
robot work is authorized by this device milestone.
