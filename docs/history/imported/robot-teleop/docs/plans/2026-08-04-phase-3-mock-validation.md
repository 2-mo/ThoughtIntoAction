# Phase 3 Mock Validation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Validate the direct ARKit-format UDP path against MockRobot, including an explicit two-sided E-stop reset and repeatable performance evidence.

**Architecture:** Add a small v1.1 reset control plane at the protocol/input/runtime boundaries while leaving the canonical core and Pose layout unchanged. A local application reset call remains the sole authority that clears a latched Mock session; the iPhone unlocks only after its matching acknowledgement.

**Tech Stack:** Python 3.12, pytest, strict mypy, Ruff, Swift 6, SwiftUI, XCTest, ARKit, Network.framework, fixed-size UDP packets.

---

### Task 1: Freeze Phase 3 design and E-stop decision

**Files:**
- Create: `docs/plans/2026-08-04-phase-3-mock-validation-design.md`
- Create: `docs/plans/2026-08-04-phase-3-mock-validation.md`
- Create: `docs/adr/0009-two-sided-estop-reset.md`
- Modify: `docs/adr/README.md`
- Modify: `TODO.md`

**Step 1: Record the failure-safe handshake and Mock-only recovery boundary.**

Document that a ResetRequest never clears Ubuntu and that a completed
acknowledgement is emitted only by explicit local reset after a safe release.

**Step 2: Mark the accepted Phase 2 device work as complete with deferred human observations.**

Move thermal/battery and ergonomics notes to follow-up observations so only
Phase 3 is in progress.

**Step 3: Run the documentation checks.**

Run: `git diff --check`

Expected: no whitespace errors.

**Step 4: Commit.**

```bash
git add docs TODO.md
git commit -m "docs: define phase three mock validation"
```

### Task 2: Add v1.1 reset request and acknowledgement packets

**Files:**
- Modify: `src/robot_teleop/protocol/version.py`
- Modify: `src/robot_teleop/protocol/packet.py`
- Modify: `src/robot_teleop/protocol/golden.py`
- Modify: `tests/unit/test_packet_codec.py`
- Modify: `tests/golden/test_golden_vectors.py`
- Create: `tests/golden/estop_reset_request_v1.bin`
- Create: `tests/golden/estop_reset_ack_v1.bin`
- Modify: `tests/golden/manifest.json`
- Modify: `docs/protocol.md`

**Step 1: Write failing tests.**

Cover 36-byte round trips, source/session validation, v1.0 decode compatibility,
unknown future minor rejection, and packets with illegal flags.

**Step 2: Run focused tests and confirm the new packets are unsupported.**

Run: `uv run pytest tests/unit/test_packet_codec.py tests/golden -q`

Expected: failure before packet implementation.

**Step 3: Implement the minimal additive codec.**

Define `EStopResetRequestPacket` and `EStopResetAckPacket`; preserve the header
layout, use dedicated packet types, accept protocol minor zero through one, and
require zero flags.

**Step 4: Regenerate the checked-in fixtures.**

Run: `uv run python tools/generate_golden.py`

**Step 5: Run focused tests and commit.**

Run: `uv run pytest tests/unit/test_packet_codec.py tests/golden -q`

Expected: pass.

```bash
git add src/robot_teleop/protocol tests/golden tests/unit/test_packet_codec.py docs/protocol.md
git commit -m "feat: add estop reset protocol"
```

### Task 3: Correlate reset readiness and require an explicit local reset

**Files:**
- Create: `src/robot_teleop/runtime/estop_reset.py`
- Modify: `src/robot_teleop/inputs/arkit.py`
- Modify: `src/robot_teleop/runtime/receiver.py`
- Modify: `src/robot_teleop/runtime/servo_loop.py`
- Modify: `src/robot_teleop/runtime/application.py`
- Modify: `src/robot_teleop/robots/mock.py`
- Modify: `src/robot_teleop/registry.py`
- Test: `tests/unit/test_estop_reset.py`
- Test: `tests/contract/test_arkit_input_contract.py`
- Test: `tests/integration/test_arkit_udp_to_mock.py`

**Step 1: Write failing tests for a request that cannot reset automatically.**

Assert an active-session request is pending, a non-E-stop released Pose makes it
ready, a missing release blocks reset, a different session invalidates it, and
only `TeleopApplication.reset_estop()` sends ResetAck and re-arms Mock.

**Step 2: Run focused tests.**

Run: `uv run pytest tests/unit/test_estop_reset.py tests/integration/test_arkit_udp_to_mock.py -q`

Expected: failure before the gate and lifecycle API exist.

**Step 3: Implement the smallest runtime gate.**

Keep only the latest request and safe release under a lock. Call the gate after
input validation; do not add socket or packet imports to `core/`.

**Step 4: Add explicit Mock re-arm injection.**

Make `MockRobotBackend.rearm_after_safe_stop()` stationary and idempotent. Pass
it only from the Mock registry composition root; no real backend is added.

**Step 5: Run focused tests and commit.**

Run: `uv run pytest tests/unit/test_estop_reset.py tests/contract/test_arkit_input_contract.py tests/integration/test_arkit_udp_to_mock.py -q`

Expected: pass.

```bash
git add src/robot_teleop tests
git commit -m "feat: require explicit estop reset"
```

### Task 4: Mirror the reset contract in the iPhone client

**Files:**
- Modify: `apps/ios/RobotTeleop/Protocol/ProtocolModels.swift`
- Modify: `apps/ios/RobotTeleop/Protocol/PacketCodec.swift`
- Modify: `apps/ios/RobotTeleop/App/ControlStateStore.swift`
- Modify: `apps/ios/RobotTeleop/App/TeleopCoordinator.swift`
- Modify: `apps/ios/RobotTeleop/App/AppModel.swift`
- Modify: `apps/ios/RobotTeleop/UI/ControlComponents.swift`
- Modify: `apps/ios/RobotTeleop/Tests/PacketCodecTests.swift`
- Modify: `apps/ios/RobotTeleop/Tests/ControlStateTests.swift`
- Modify: `apps/ios/RobotTeleop/Tests/CoordinatorTests.swift`

**Step 1: Write failing XCTest cases.**

Cover golden bytes, a reset request followed by an E-stop-clear released Pose,
no UI unlock for a mismatched ack, reconnect re-latching while pending, and
unlock only for the matching acknowledgement.

**Step 2: Run the focused XCTest classes.**

Run: `make ios-test`

Expected: failure before the new models and states exist.

**Step 3: Implement the client state machine.**

Keep the visible lock while sending the release proof. Retry the bounded
low-rate request; do not make Pose reliable or add a timer that resends ARFrame
data.

**Step 4: Run Swift formatting and tests.**

Run: `make ios-test`

Expected: pass.

**Step 5: Commit.**

```bash
git add apps/ios tests/golden
git commit -m "feat: add ios estop reset handshake"
```

### Task 5: Build repeatable Mock validation and operator tooling

**Files:**
- Create: `src/robot_teleop/tools/phase3_mock_validation.py`
- Create: `tools/phase3_mock_validation.py`
- Create: `tools/phase3_mock_session.py`
- Create: `tests/integration/test_phase3_mock_validation.py`
- Modify: `Makefile`
- Modify: `README.md`
- Modify: `docs/performance.md`

**Step 1: Write a failing deterministic end-to-end scenario test.**

Exercise axis signs/scales/orientation, Clutch/Recenter continuity, tracking
and session recovery, invalid input, watchdog, reset, and latest-only burst
delivery through real UDP sockets and MockRobot.

**Step 2: Run the new integration test.**

Run: `uv run pytest tests/integration/test_phase3_mock_validation.py -q`

Expected: failure before the scenario/report exists.

**Step 3: Implement the finite report and local operator command loop.**

Emit JSON only after completion. The session helper must never enable hardware;
its only mutation is the explicit Mock reset call after a typed local command.

**Step 4: Run a short and a configurable stability soak.**

Run: `uv run python tools/phase3_mock_validation.py --duration 2`

Expected: structured report with bounded histories, no receiver/servo fault,
and no hardware enabled.

**Step 5: Commit.**

```bash
git add src/robot_teleop/tools tools tests/integration Makefile README.md docs/performance.md
git commit -m "feat: add phase three mock validation"
```

### Task 6: Quality gate and Phase 3 report

**Files:**
- Create: `docs/phase3_report.md`
- Modify: `TODO.md`
- Modify: `README.md`
- Modify: `docs/architecture.md`
- Modify: `apps/ios/README.md`

**Step 1: Record automated evidence and remaining operator-only gates.**

Do not mark physical axis review or ten-minute iPhone evidence complete until
the operator performs it.

**Step 2: Run all repository quality checks.**

Run: `make check && make ios-test && git diff --check`

Expected: all local static checks and tests pass.

**Step 3: Commit and push the feature branch.**

```bash
git add README.md TODO.md docs apps/ios/README.md
git commit -m "docs: report phase three mock validation"
git push
```

