# iOS Capture and Status Design

Date: 2026-08-04

Status: approved for implementation.

## Problem

The first physical iPhone validation confirmed a stable direct 60 Hz ARKit UDP
stream, but the app visibly warmed the iPhone and the `Disconnect` command was
clipped. The initial implementation also published a full SwiftUI status update
for every Pose packet. That creates main-actor and view invalidation work at the
same cadence as ARKit even though display telemetry is not part of the control
path.

The user prioritizes response and smoothness. A 120 Hz display must therefore
not be treated as a 120 Hz ARKit pose source. ARKit exposes its independent
camera formats through `ARWorldTrackingConfiguration.supportedVideoFormats`.
The application must measure and use the real device capability rather than
assume it from ProMotion.

## Goals

- Preserve one UDP Pose packet for every new ARKit frame and never resend an old
  pose merely to claim a higher rate.
- Select the highest available ARKit video cadence at runtime. For formats with
  the same cadence, select the smallest image resolution to avoid needless
  capture work.
- Keep the Ubuntu latest-only Servo at its configured 200 Hz. A real 120 Hz
  input source naturally gives it fresher targets; a 60 Hz source still has a
  200 Hz local control cadence without a historical queue.
- Reduce non-control UI work to at most 10 Hz during steady Pose streaming while
  publishing tracking transitions, networking state, errors, RTT, and settings
  acknowledgements immediately.
- Explicitly disable unused ARKit capture work: light estimation, HDR,
  environment textures, frame semantics, and audio.
- Show the selected capture format and measured ARFrame cadence so physical
  testing proves whether this iPhone supplies 60 Hz or 120 Hz ARKit frames.
- Give the `Disconnect` command a stable width that fits its label in portrait
  and landscape.

## Non-goals

- No synthetic 120 Hz packet timer, interpolation sender, CoreMotion fusion, or
  custom VIO implementation in this change.
- No change to binary packet formats, Ubuntu safety logic, receiver semantics,
  Servo rate, RobotBackend, CAN, or hardware behavior.
- No attempt to force SwiftUI or ProMotion display rendering to 120 Hz. Screen
  refresh does not create new ARKit measurements and would work against the
  thermal observation.

## Runtime flow

```text
ARKit camera at measured supported cadence (for example 60 or 120 Hz)
    -> ARSessionDelegate.didUpdate(frame)
    -> one canonical Pose packet per new frame
    -> latest-only UDP transport
    -> Ubuntu receiver and 200 Hz latest-only Servo

Pose telemetry
    -> coordinator status coalescer (10 Hz maximum)
    -> MainActor / SwiftUI metrics
```

ARKit callbacks, packet encoding, UDP sending, and cadence measurement remain
off the main actor. The status coalescer is only a UI-observation boundary; it
does not delay, filter, or suppress Pose packets.

## Capture-format selection

`ARKitPoseProvider` creates the configuration once per session start. It ranks
the formats supplied by the device as follows:

1. highest `framesPerSecond`;
2. lowest `imageResolution.width * imageResolution.height` among formats at
   that frame rate;
3. original device order as a deterministic tie-breaker.

If ARKit offers 120 Hz on the iPhone, it is selected and packet cadence follows
it. If it only offers 60 Hz, the app reports 60 Hz and keeps the existing fresh
frame path. The selected value and an interval-window measured rate are exposed
as capture telemetry. The measured value, not the display refresh rate, is the
acceptance signal.

## Status coalescing

`TeleopCoordinator` owns a monotonic-clock throttle. Ordinary pose-driven
status changes may call the status handler once each 100 ms. It always retains
the newest sequence, send rate, tracking value, and replacement count before
the next publication. These events bypass the throttle:

- first status and connect/disconnect/transport transitions;
- tracking quality transition, including unavailable;
- malformed response accounting and transport error;
- Pong/RTT and Settings acknowledgement or rejection;
- control safety edges such as clutch release and E-stop.

The status handler remains safe to call from the coordinator queue. `AppModel`
continues to hop it onto `MainActor`; that hop now happens at a bounded UI rate
under steady streaming.

## Validation

Automated tests cover deterministic format selection, configuration feature
flags, Pose delivery during status coalescing, immediate tracking publication,
and the `Disconnect` control width. Simulator verification checks layout and
the unchanged protocol path. Physical validation runs a 3-minute foreground
session on the iPhone 16 Pro Max, recording selected capture FPS, measured
ARFrame FPS, UDP Pose FPS, RTT, and an Xcode Power Profiler trace. The user then
compares hand temperature and follow feel with the previous build.

No result will claim 120 Hz until the physical device reports it.
