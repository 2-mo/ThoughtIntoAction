# Phase 4 Ghost Visualization Plan

Date: 2026-08-04

Status: In progress

## Scope

Phase 4 starts with a hardware-free Ghost review artifact, then adds an offline
X5 model preflight. The first cut focuses on direction, scale, and orientation
mapping; the second verifies representative relative targets with FK, Jacobian,
and bounded seeded IK before any robot backend exists.

## Implementation Steps

1. Generate a static Mock and Ghost review from the active profile.
2. Show configured axis signs, position scale, rotation scale, and workspace.
3. Render input-axis translation projections and mapped orientation axes.
4. Reuse the canonical pipeline and MockRobot to export target/TCP trajectories,
   Clutch/Recenter/release transitions, and workspace safety rejects.
5. Keep all output offline in build output and outside Servo, Receiver, and core.
6. Add tests that verify mapping signs and pipeline trajectory continuity.
7. Add a source-attributed X5 URDF and optional Pinocchio adapter outside core.
8. Check a deterministic relative Clutch/Recenter/rotation trajectory from a
   synthetic offline seed, recording IK residuals, joint steps, and Jacobian
   diagnostics.

## Explicit Non-goals

- No robot hardware, CAN, ARX5 SDK, online Ruckig, or live-path IK.
- No live visualization process in the Servo thread.
- No MeshCat or MuJoCo dependency until the static review proves insufficient.
- No claim that visual review replaces later physical safety validation.

## Exit Criteria

- The operator can open one local HTML file and review the configured mapping.
- The generated JSON is deterministic and test-covered, including target/TCP
  trajectory and safety reject data.
- The optional model report is deterministic, uses the cited X5 model asset,
  and cannot construct hardware components.
- The roadmap keeps long connected-iPhone soak testing as a pre-hardware gate,
  not as a blocker for Mock-only visualization work.
