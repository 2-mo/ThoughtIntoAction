# iOS Capture and Status Implementation Plan

> **For Codex:** Execute this plan on the existing `feat/phase-2-device-validation` branch. Do not create a new worktree, connect robot hardware, or merge to `main` without explicit user acceptance.

**Goal:** Reduce unnecessary iPhone work without sacrificing fresh ARKit Pose delivery, select and measure the best real ARKit capture cadence, and correct the clipped disconnect command.

**Architecture:** ARKit continues to emit one canonical sample per new `ARFrame`; the coordinator sends it immediately through the existing latest-only UDP transport. A separate monotonic status coalescer bounds only MainActor/SwiftUI telemetry updates. The existing 200 Hz Ubuntu Servo is unchanged and consumes the most recent fresh target.

**Tech Stack:** Swift 6, SwiftUI, ARKit, Network.framework, XCTest, Xcode 26.6, iPhone 16 Pro Max on iOS 26.5.2, Python 3.12, pytest, Ruff, and mypy.

---

### Task 1: Lock down capture and status behavior with tests

**Files:**
- Modify: `apps/ios/RobotTeleop/Tests/CoordinatorTests.swift`
- Modify: `apps/ios/RobotTeleop/Tests/ProviderTests.swift`
- Modify: `apps/ios/RobotTeleop/Tests/RobotTeleopUITests.swift`

**Steps:**
1. Add a test showing a burst of new Pose samples still produces every packet
   while status callbacks are bounded by an injected interval.
2. Add a test showing a tracking transition bypasses that interval.
3. Add pure format-ranking tests for highest FPS, lower-pixel tie-break, and
   deterministic fallback.
4. Assert the ARKit configuration explicitly disables unused capture features.
5. Assert the connected-state `Disconnect` button is at least 120 points wide.
6. Run the focused XCTest cases and confirm the new expectations fail before the
   implementation changes.

### Task 2: Implement bounded UI status delivery

**Files:**
- Modify: `apps/ios/RobotTeleop/App/TeleopCoordinator.swift`
- Modify: `apps/ios/RobotTeleop/App/AppModel.swift`

**Steps:**
1. Inject a default 100 ms status publication interval into the coordinator.
2. Preserve the newest telemetry state while suppressing only ordinary
   pose-driven callbacks within that interval.
3. Force publication on status transitions and safety/control edges.
4. Add capture telemetry to the status model and preserve MainActor-only model
   mutation.
5. Re-run the coordinator tests; expect all pose and safety tests to pass.

### Task 3: Implement real capture-cadence selection and telemetry

**Files:**
- Modify: `apps/ios/RobotTeleop/Tracking/ARKitPoseProvider.swift`
- Modify: `apps/ios/RobotTeleop/Tracking/PoseProvider.swift`
- Modify: `apps/ios/RobotTeleop/App/TeleopCoordinator.swift`
- Modify: `apps/ios/RobotTeleop/UI/StatusStrip.swift`

**Steps:**
1. Rank the device-provided ARKit video formats by highest FPS, then lowest
   pixel count, with stable original order as tie-breaker.
2. Explicitly disable light estimation, HDR, audio, frame semantics, and
   environment texturing while retaining world tracking and autofocus defaults.
3. Measure ARFrame cadence over a short interval window without a timer or
   duplicate pose samples.
4. Send selected and measured source cadence to the coordinator/UI without
   affecting Pose send timing.
5. Re-run provider and protocol tests; expect no packet layout changes.

### Task 4: Correct the compact header layout

**Files:**
- Modify: `apps/ios/RobotTeleop/UI/ControlComponents.swift`
- Modify: `apps/ios/RobotTeleop/Tests/RobotTeleopUITests.swift`

**Steps:**
1. Give the connection command a stable 120-point width at both size classes.
2. Verify portrait and landscape screenshots with the connected state visible.
3. Confirm no header field or status content overlaps at normal Dynamic Type.

### Task 5: Verify, deploy, and document the physical result

**Files:**
- Modify: `docs/performance.md`
- Modify: `docs/phase2_report.md`
- Modify: `TODO.md`

**Steps:**
1. Run Swift tests/build, Python quality gates, architecture checks, and
   `git diff --check`.
2. Install the signed debug build on the connected iPhone; do not connect to a
   robot or start a hardware backend.
3. Capture a three-minute Power Profiler trace while the user runs direct
   iPhone-to-Ubuntu Mock UDP, then collect selected/measured capture FPS and
   UDP Pose FPS.
4. Record only measured results, including whether ARKit actually reaches
   120 Hz, and leave user hand-feel acceptance pending until reported.
5. Commit and push the feature branch. Wait for CI and explicit user acceptance
   before any merge to `main`.
