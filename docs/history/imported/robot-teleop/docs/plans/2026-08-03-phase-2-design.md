# Phase 2 Simulator-First iOS Design

Date: 2026-08-03

Status: approved for implementation. Real-device ARKit validation remains
pending until the iPhone is connected.

## Scope and confirmed environment

Phase 2 builds the Swift/SwiftUI ARKit input client only. The app sends directly
to Ubuntu and contains no robot model, IK, Ruckig, CAN, workspace, robot safety
envelope, or robot SDK code. The first deployment target is an iPhone 16 Pro Max
running iOS 26.5.2. The project uses an iOS 26.0 deployment target, Xcode 26.6,
the installed iOS 26.5 simulator runtime, bundle identifier
`com.mo.robotteleop`, and Automatic Signing with the user's free Personal Team.

The first endpoint defaults to `100.112.78.10:5005`. It is a Tailscale address
reachable directly from the iPhone. Tailscale supplies routing only; the Mac is
never a runtime hop. Host and port remain editable deployment values and are not
compiled into the protocol or canonical core.

## Application boundaries

The app has four explicit boundaries:

```text
ARKitPoseProvider / SimulatedPoseProvider
                 -> TeleopCoordinator
                 -> PacketCodec
                 -> LatestDatagramUDPClient
                 -> Ubuntu
```

`PoseProvider` emits a new immutable sample only when new source data exists.
The real provider emits at most once per `ARSessionDelegate.didUpdate` frame.
The simulator provider creates new synthetic samples and never repeats an old
pose on a faster timer. Both feed the same coordinator, codec, metrics, and UDP
client so simulator testing covers the real control path below ARKit.

UI state is owned on `MainActor`. Small control snapshots cross to the tracking
queue through a lock-protected store. ARKit callbacks, packet encoding, and
Network.framework operations stay off the main thread. The UDP sender permits
one Pose send in flight and one replaceable pending Pose; a newer frame
overwrites pending work. Low-rate control packets are never mixed into a
historical Pose queue.

## Protocol and dynamic scale control

Existing Pose, Ping, and Pong layouts remain byte-for-byte unchanged. Swift
loads the checked-in Python golden files as test resources and must reproduce
all bytes, including little-endian integers, Float32 values, and `xyzw`
quaternions. Pose uses a monotonic sequence stream. Ping uses a separate request
sequence echoed by Pong so a low-rate clock exchange cannot appear as Pose loss.

Position and rotation sliders cannot scale ARKit world poses locally because
that would duplicate canonical mapping. Phase 2 therefore adds a low-rate,
generic Settings/SettingsAck pair containing only position and rotation scales.
Ubuntu accepts Settings only for the active Pose session and returns the values
it applied. A thread-safe mapping-settings store supplies the common relative
pose policy. The policy snapshots settings only on clutch activation or
recenter, so changing sensitivity cannot jump an active target. Existing robot
target limits remain authoritative.

Pose has no ACK or retransmission. Ping/Pong remains low-rate. Settings is sent
after the first Pose and when slider editing ends; the UI can distinguish
pending from acknowledged settings.

## Tracking and controls

`ARWorldTrackingConfiguration` is used without plane detection, scene
reconstruction, environment texturing, video streaming, or rendering content.
Camera transform translation is metres. `simd_quatf` is serialized as
`imag.x, imag.y, imag.z, real`. Tracking maps to normal, limited, or unavailable.
Session interruption and failure immediately publish unavailable tracking.

Clutch is a momentary press gesture. Ending the gesture, losing scene activity,
disconnecting, tracking loss, or app backgrounding clears it immediately and
sends the newest available released state. Recenter is consumed by exactly one
new Pose. E-stop is locally latched, clears Clutch, and remains requested in
subsequent Pose packets. The app provides no remote E-stop reset.

Gripper and scale controls are disabled while E-stop is latched. Scale editing
is disabled while Clutch is pressed. Haptics acknowledge clutch edges,
connection changes, and E-stop requests but never gate networking.

## UI design

The first screen is the operational controller, not a landing page. It uses a
quiet charcoal work surface with green, amber, red, and blue state accents. A
compact header exposes connection state and endpoint editing. Stable metric
cells show tracking, send FPS, sequence, RTT, session, and network state without
resizing. Gripper and sensitivity use sliders. Recenter uses the familiar scope
symbol, and E-stop uses a stop symbol plus an explicit label.

The primary control is a large, fixed-height Hold to Control surface placed for
thumb reach. The layout adapts between portrait and landscape; landscape is
preferred on the Pro Max, while portrait remains usable. Controls carry
accessibility labels, values, and minimum touch targets. No status text overlaps
the control surface at Dynamic Type sizes used in normal operation.

## Failure behavior

Malformed Pong or SettingsAck packets are ignored and counted. Connection
failure releases Clutch, stops tracking transmission, cancels timers, clears
pending Pose work, and leaves the UI disconnected. Reconnect creates a new
nonzero session ID and sends a released Pose before any possible activation.
App backgrounding follows the same release-first rule.

The simulator cannot prove ARKit camera access, tracking cadence, thermal
behavior, real Tailscale UDP, haptics, or true-device UI ergonomics. Phase 2 is
not complete until the connected iPhone builds, runs, passes Swift golden tests,
and sends ARKit data directly to Ubuntu MockRobot. Simulator work can complete
all other implementation and automated checks first.

## Verification

XCTest covers codec errors and golden bytes, clock calculations, control
one-shot/latch behavior, latest-only sender replacement, session regeneration,
and simulator sample monotonicity. Python tests cover the additive protocol and
dynamic mapping behavior. A full simulator build and test run uses an iOS 26.5
Pro Max destination. Screenshots are inspected in portrait and landscape for
blank content, clipping, unstable dimensions, and overlap.

The existing Python Ruff, formatting, strict mypy, architecture, contract, and
integration suites remain mandatory. No test, preview, or app launch connects
to ARX5 or CAN.
