# Pose Monitor Control Observability Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Extend the hardware-free Pose monitor with reusable observations for
physical input controls so Phase 2 device behavior can be accepted without
changing the wire protocol or control path.

**Architecture:** A small tool-layer accumulator consumes canonical
`TeleopInput` values and reports control edges, repeated states, gripper range,
and tracking distribution. `pose_monitor` remains the ARKit UDP composition
root, while the accumulator itself has no ARKit, UDP, Servo, or robot
dependency and can be reused by future VR and local input diagnostics.

**Tech Stack:** Python 3.12, frozen canonical models, pytest, Ruff, and strict
mypy.

---

### Task 1: Specify canonical control observations

**Files:**
- Create: `tests/unit/test_control_observations.py`
- Create: `src/robot_teleop/tools/control_observations.py`

**Steps:**
1. Write failing tests for Clutch activation/release edges, one-shot Recenter,
   E-stop persistence, gripper samples/range/changes, and tracking counts.
2. Cover session changes so reconnects report controls that begin active
   separately instead of creating false activation or release edges.
3. Implement a bounded accumulator that retains only counts and latest scalar
   state, never packet history.
4. Run the focused tests and expect PASS.

### Task 2: Integrate the report into Pose monitor

**Files:**
- Modify: `src/robot_teleop/tools/pose_monitor.py`
- Modify: `tests/integration/test_tools.py`

**Steps:**
1. Observe each accepted canonical input exactly once.
2. Add a stable `control_observations` object to the JSON report without
   changing existing fields or exit behavior.
3. Verify simulator-to-monitor integration and the existing Settings/Ping/Pong
   path.

### Task 3: Verify, publish, and deploy

**Files:**
- Preserve and review: `.gitignore`
- Preserve and review: `apps/ios/RobotTeleop/RobotTeleop.xcodeproj/project.pbxproj`

**Steps:**
1. Run focused tests, then Ruff, Ruff format check, strict mypy, and all pytest.
2. Run Swift format, simulator build, and XCTest because Xcode signing changed
   the project file during physical-device setup.
3. Commit the diagnostics and reviewed Xcode device configuration separately,
   then push the feature branch.
4. Fast-forward the hardware-free Ubuntu checkout and run the monitor using
   system Python plus `PYTHONPATH`; do not connect a robot or start CAN.
5. Observe Clutch, Recenter, gripper, E-stop, tracking, cadence, loss, and RTT on
   the connected iPhone, then update the Phase 2 report and roadmap.
