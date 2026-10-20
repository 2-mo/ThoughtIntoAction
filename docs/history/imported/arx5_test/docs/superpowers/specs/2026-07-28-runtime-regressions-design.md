# Runtime Regressions Design

## Context

Three runtime symptoms remain visible in the Web console:

- RGB continues working while the depth view reports unavailable.
- The TCP editor and robot model can stop following measured motion.
- The simulation can appear frozen during joint reset or park execution.

The running `web-lan` process predates the current depth-recovery code. Once
that process has fallen back to RGB-only, it cannot use the newer retry logic
until restarted. The current controller already keeps a lock-protected snapshot
from the motion monitor, but the Web server exposes it only when the browser's
generic state poll arrives. This makes motion visualization depend on browser
timer scheduling and one outstanding polling request. Path rendering is already
bounded, so feedback delivery is the remaining shared failure point for the TCP
and park symptoms.

## Goals

- Keep RGB usable when the D455 depth stream fails to start or drops out.
- Retry depth initialization without requiring a page reload or camera toggle.
- Stream monitor-owned arm snapshots during all long-running Web motions.
- Keep the TCP controls, joint model, and executed path aligned with measured
  feedback throughout reset and park motions.
- Make stale service processes visible so deployed behavior can be tied to the
  code revision actually running.

This change does not add a second SDK reader, alter motion planning, weaken
safety checks, or change joint, Cartesian, gripper, or collision limits.

## Approach

The controller motion monitor remains the only live SDK feedback reader during
motion. It continues converting each validated joint sample into a cached
`ArmSnapshot`, including an FK-derived TCP pose.

`WebArmSession.run_operation` will publish `motion_state` at the existing 40 ms
feedback period. Each update reads only `controller.get_cached_state()` and
therefore cannot race the SDK reader. The generic browser state poll remains in
place for idle state and as a connection watchdog, but it is no longer the
primary motion transport.

The browser will accept both `motion_state` and normal `state` payloads through
the same `updateState` path. Motion-start and motion-finish events continue to
control TCP feedback-follow mode. Executed-path accumulation stays capped at
360 points, geometry rebuilds stay limited to one per 80 ms, and tube geometry
remains disabled while motion is active.

The camera worker will retain the current staged recovery behavior: attempt
RGB-D first, fall back to RGB-only on a depth-specific failure, release all
pipeline-owned objects, and retry RGB-D with bounded exponential backoff. A
successful retry replaces the black fallback depth frames with live colorized
depth frames on the existing WebRTC track. A failed retry must leave RGB and
the data channel alive.

## Data Flow

Arm motion feedback follows one ownership chain:

1. The controller monitor reads one SDK joint state.
2. Safety checks validate that state.
3. The monitor computes TCP FK and replaces the cached snapshot.
4. The Web operation task reads the cache every 40 ms and sends `motion_state`.
5. The browser updates measured joints, TCP pose, controls, and path progress.
6. The final command result supplies one authoritative post-motion snapshot.

Depth recovery follows a separate state machine:

1. Start one RGB-D pipeline.
2. On depth startup, initialization, or sustained-frame failure, release it.
3. Start RGB-only and publish the depth error plus next retry delay.
4. At the retry deadline, release RGB-only and attempt RGB-D again.
5. On success, clear the error and continue through the same WebRTC tracks.
6. On failure, restore RGB-only and schedule the next bounded retry.

## Error Handling

A missing cached arm snapshot skips one `motion_state` interval; it never falls
back to a concurrent SDK read. Motion safety errors retain existing damping and
fault handling. The last cached snapshot is included in failure state whenever
available so the browser does not jump back to a stale pose.

Depth errors are reported as degraded camera status, not total camera failure,
while RGB is healthy. Failure to restore RGB-only after a retry is a full camera
failure and follows the existing supervisor restart path. Retry delay is capped
at 30 seconds and resets after successful RGB-D recovery.

The control hello and camera status payloads will expose both
`source_revision` and `service_started_at`. A missing Git revision is reported
as `unknown`; the process start timestamp is always present. The browser may
display these values in diagnostics, and automated tests prove both fields are
present. This does not auto-restart a service during robot operation.

## Verification

- A controller regression test proves monitored feedback updates the cached TCP
  pose without another SDK read.
- A Web regression test proves `motion_state` arrives before `command_result`
  and that live `get_state()` is never called while the worker owns feedback.
- Frontend contract tests prove `motion_state` uses the normal state-update path.
- Camera state-machine tests cover initial depth failure, failed retry, restored
  RGB fallback, successful retry, and retry-delay reset.
- Source tests preserve the 360-point cap, 80 ms geometry throttle, and no tube
  rebuild during active motion.
- The full non-hardware suite and JavaScript syntax checks pass.
- Browser verification exercises synthetic long-motion feedback at desktop and
  mobile sizes, checks a nonblank WebGL canvas, and checks console errors.
- Hardware verification restarts `web-lan`, confirms RGB remains live during a
  forced depth fallback, then confirms depth recovers without reloading.

## Deployment

Implementation is committed only on `fix/runtime-regressions`. Existing
`web-lan` processes are not restarted during coding because stopping the control
service can change the robot's control mode. After tests pass, restart requires
an explicit operator-safe window, followed by browser cache revalidation and
the hardware checks above.
