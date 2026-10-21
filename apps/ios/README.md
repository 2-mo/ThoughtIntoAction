# iOS application

The Phase 2 app lives in `RobotTeleop/RobotTeleop.xcodeproj`. Simulator builds
select `SimulatedPoseProvider`; physical-device builds select
`ARKitPoseProvider`. Both feed the same coordinator, binary codec, latest-only
UDP sender, controls, and metrics. ARKit emits at most one sample for each new
`ARFrame` and no timer resends an old camera pose.

The project targets iOS 26.0, Swift 6, Xcode 26.6, and the iOS 26.5 runtime.
Warnings are errors. The first physical target is an iPhone 16 Pro Max on iOS
26.5.2 with bundle identifier `com.mo.robotteleop`.

## Simulator

The portable test destination is the installed iPhone 17 Pro Max simulator:

```bash
make ios-test
```

An exact local iPhone 16 Pro Max simulator can be selected with:

```bash
make ios-test \
  IOS_DESTINATION='platform=iOS Simulator,name=RobotTeleop iPhone 16 Pro Max,OS=26.5'
```

Run the real Swift-to-Python UDP loopback with:

```bash
make ios-network-test
```

Exercise the same downstream path with a 120 Hz simulator Pose source with:

```bash
make ios-network-test-120
```

This is a transport and receiver readiness check only. It does not claim that a
physical ARKit session produces 120 fresh `ARFrame` poses; that remains a
connected-device measurement.

The app is launched against `127.0.0.1:5005` only for that UI test. A normal app
launch defaults to `100.112.78.10:5005`, remains disconnected until Connect is
pressed, and permits editing both fields.

## Personal Team device install

A paid developer membership is not required for testing on the owner's iPhone.
Apple calls the free signing identity a Personal Team. Its provisioning profile
expires after seven days, so the app must be rebuilt and reinstalled
periodically. Personal Team signing is for personal device testing, not App
Store distribution.

For a connected device:

1. Connect and trust the iPhone, then enable Developer Mode if iOS requests it.
2. Open the project, select the `RobotTeleop` target, and leave Automatically
   manage signing enabled.
3. Select the signed-in Personal Team and keep `com.mo.robotteleop` unless Xcode
   reports that the identifier is unavailable.
4. Select the connected iPhone 16 Pro Max as the run destination and press Run.
5. Grant Camera and Local Network access on first launch.
6. On Ubuntu, run the hardware-free monitor shown below.
7. Confirm the app endpoint is `100.112.78.10:5005`, press Connect, and verify
   NORMAL tracking, nonempty sequence/FPS, RTT, and SYNCED settings.

```bash
uv run python tools/pose_monitor.py \
  --bind-host 0.0.0.0 \
  --port 5005 \
  --duration 60 \
  --exit-after-round-trip
```

This test sends only input data to a hardware-free monitor. It does not connect
to MockRobot, ARX5, CAN, or any robot SDK.

E-stop is intentionally latched. Disconnect/Connect and returning from the
home screen do not clear it. Pressing `REQUEST RESET` sends a session-scoped
v1.1 reset request and a released, normal-tracking Pose, but the app remains
locked in `RESET PENDING`. Ubuntu must explicitly complete its reset authority
before it returns an ACK with the same source, session, sequence, and timestamp.
Only that matching ACK unlocks the app; a phone-only unlock is not a safe reset
protocol.

The current reset composition is Mock-only. For a connected-iPhone Phase 3
session, run this local Ubuntu console and type `reset` only after the app is in
`RESET PENDING` and the operator has checked the situation:

```bash
./run.sh components run diagnostic.phone_reset_console
```

The console cannot enable a real backend, initialize CAN, or substitute for a
physical emergency stop. Connected-device reset and coordinate validation remain
open Phase 3 checklist items.

Apple's current Personal Team limits are documented at
<https://developer.apple.com/help/account/basics/about-your-developer-account>.
Protocol and frame contracts remain in `docs/protocol.md` and
`docs/coordinate_frames.md`. Completed and remaining physical-device checks are
tracked in `docs/phase2_report.md` and `TODO.md`.
