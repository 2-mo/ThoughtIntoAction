# Performance Measurement

Phase 1 establishes an instrumented latest-only Ubuntu path. Phase 2 adds the
bounded iOS sender and client-side cadence/RTT metrics. Neither simulator result
is a real-time or Wi-Fi latency guarantee.

## Objective

Low latency and smooth, direct response are cross-cutting design goals, not work
deferred until Phase 7. Within verified RobotBackend hard limits, prefer the
configuration with the lowest input-to-command P95/P99 latency and Servo jitter
that also produces continuous target, velocity, and acceleration behavior.

The initial 200 Hz loop and current Watchdog thresholds are baselines. They may
be increased, reduced, or structurally replaced after measurement. A safety
policy is not accepted merely because it is conservative if it adds avoidable
phase lag, stop-start behavior, or clutch interruptions without improving a
measured failure outcome.

## Servo measurements

- expected deadline, actual start, and actual end;
- compute time and cycle period;
- lateness/jitter;
- deadline miss and consecutive misses;
- deadline rebases after falling behind multiple periods;
- feedback timestamp progress.

## Input measurements

- accepted and rejected samples;
- duplicate, out-of-order, stale, and invalid counts;
- latest host receipt age;
- session changes, estimated packet loss, superseded datagrams, and rejects by
  stable protocol reason;
- low-rate Ping/Pong RTT and clock-offset samples.

## iOS capture cadence and thermal behavior

ProMotion display refresh is not an ARKit pose-sampling guarantee. The iOS
client chooses from `ARWorldTrackingConfiguration.supportedVideoFormats` at
runtime: highest available camera FPS first, then lowest image resolution among
formats at that FPS. It emits exactly one Pose for each new `ARFrame`. A source
that truly produces 120 Hz frames therefore supplies 120 fresh Pose targets;
the existing 200 Hz Ubuntu latest-only Servo can consume them without a rate
change. A source that produces 60 Hz frames remains 60 Hz rather than repeating
old poses on a synthetic 120 Hz timer.

The app exposes the selected camera format and a measured ARFrame cadence in
the endpoint status line. The existing Send FPS metric remains the outgoing
Pose cadence. A 120 Hz result is accepted only when the physical phone reports
it through those measurements, not because its display supports ProMotion.

On the iPhone 16 Pro Max running iOS 26.5.2, the physical App selected
`60 Hz 1280x720` and measured `60.0 FPS`. The direct iPhone-to-Ubuntu sample
therefore validates a 60 Hz source for this configuration. Do not add a timer
that resends an old ARKit pose at 120 Hz to match the display refresh rate.

Steady Pose traffic may update SwiftUI telemetry at most every 100 ms. Tracking
changes, connection state, errors, RTT, settings acknowledgements, Clutch
release, and E-stop edges bypass that UI-only throttle. Pose encoding and UDP
sending are never throttled by it.

For this no-rendering input app, ARKit explicitly disables plane detection,
environment texturing, HDR environment textures, light estimation, audio, and
frame semantics. Camera autofocus remains at its ARKit default. Thermal work
is measured rather than inferred from these settings.

Use a foreground, direct iPhone-to-Ubuntu Mock UDP session to capture a
three-minute trace after the app is connected:

```bash
DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer \
  xcrun xctrace record \
  --template 'Power Profiler' \
  --device 'Surface Phone' \
  --attach RobotTeleop \
  --time-limit 3m \
  --output build/ios-power-profiler.trace
```

Record selected capture FPS/resolution, measured ARFrame FPS, Send FPS, RTT,
packet loss, phone temperature impression, and user follow feel alongside the
trace. Do not claim a thermal improvement until this matched physical run is
complete.

## Clock-domain rule

iPhone and Ubuntu monotonic timestamps are not directly subtractable. Ping/Pong
uses four timestamps and reports:

```text
RTT = (t3 - t0) - (t2 - t1)
server_minus_client = ((t1 - t0) + (t2 - t3)) / 2
```

High-RTT or impossible samples are rejected, and the offset is the median of a
bounded lowest-RTT subset. Until the client has a valid estimate, only Ubuntu
receipt age is reported. Phase 1 does not claim true one-way Wi-Fi latency.

## Latest-only network behavior

The UDP transport blocks for the first datagram, temporarily switches to
nonblocking mode, and drains at most the configured bound. Generic callers can
take only the newest datagram. The ARKit adapter instead processes all controls
in that finite batch while publishing only its newest valid Pose. The runtime
then overwrites one `LatestValue`; there is no command queue to accumulate
delay.

The iOS sender independently permits one Pose datagram in flight and one
replaceable pending Pose. Every new ARKit frame is encoded at most once. A slow
Network.framework completion therefore replaces obsolete pending work instead
of extending latency. Ping and Settings use low-rate matched exchanges rather
than acknowledging every Pose frame.

A regression test specifically verifies that the empty drain probe does not
wait for the main socket timeout. This caught a macOS behavior where
`MSG_DONTWAIT` combined with a configured timeout added about 100 ms per single
datagram; the implementation now uses an explicit nonblocking drain section.

## Responsiveness and smoothness measurements

- input receipt to Servo read and target generation;
- host-side accepted-Pose inter-arrival time (`input_interval_ms`) and its
  effective source cadence;
- target generation to backend command and feedback;
- target position/orientation continuity;
- commanded velocity, acceleration, and jerk continuity;
- clutch/recenter/recovery discontinuity and zero-command time;
- filter phase delay with filtering enabled and disabled;
- Watchdog intervention and recovery frequency under real network jitter.

## Reports

The metrics worker uses bounded in-memory samples. Reports include count, mean,
median, P90/P95/P99, maximum, misses, and effective loop frequency. Each report
also labels the metric window as `all_session`/`all_observed` or
`recent_window`; cumulative counters and a recent window must not be compared
as though they covered the same interval. Formatting and file output happen
only after or outside the servo thread.

## Phase 3 Mock baseline

`tools/phase3_mock_validation.py` runs a finite local UDP scenario through the
real ARKit decoder, Receiver, latest-only handoff, 200 Hz Servo, and
`MockRobotBackend`. Its JSON report records mapping/recovery checks,
Clutch-release-to-HOLD time, bounded command history, protocol rejects, input
rejects, loop frequency, jitter, packet age, and every Watchdog stage.

The scenario deliberately includes an E-stop. A reset request plus a released,
normal-tracking Pose only makes the reset gate ready; a local
`TeleopApplication.reset_estop()` call is still required before the matching ACK
is sent. The final input pause exercises the 100/200/500 ms local
HOLD/BRAKE/SAFE_STOP stages used by this test profile.

Its reported RTT is loopback process/socket latency, not Wi-Fi, Tailscale, or
one-way iPhone-to-Ubuntu latency. Use it to catch deterministic regressions and
run the iPhone Mock session separately for physical network evidence.

Required later tests include a 10-minute run, stable memory, no growing latency,
no queue backlog, packet-loss simulation, and clutch-release-to-zero-velocity
time. Filtering must be bypassable for raw latency measurement and retained only
when its smoothness gain is worth its measured phase delay.

The Phase 3 Mock console and finite validator use a bounded 131,072-sample
window. At 200 Hz it covers 10 minutes and 55 seconds, so a ten-minute report
has exact whole-session cycle and packet-age percentiles while retaining a
fixed memory ceiling.

`input_interval_ms` is measured at the Receiver handoff and is independent of
the low-rate iOS Ping/Pong RTT. A high RTT sample without a matching input
interval or packet-age spike is a diagnostic/control-plane scheduling event,
not evidence that every Pose command waited for that RTT.

Optimization starts in each phase with a latency budget. C++ migration remains
evidence-driven, but it is permitted whenever Python, GIL, IK, SDK, or scheduling
measurements show a material improvement, not only after complete failure at
200 Hz. Every change records before/after P50/P95/P99 latency, jitter, misses,
continuity, and stopping results.

Run finite local diagnostics with:

```bash
uv run python tools/udp_loopback.py --count 100
uv run python tools/network_test.py --duration 2 --rate-hz 1000
uv run python tools/pose_monitor.py --duration 5
make ios-network-test
make ios-network-test-120
```

Machine-specific Phase 1 baseline measurements are recorded in
[`phase1_report.md`](phase1_report.md). They are regression evidence only;
controller, SDK, and robot feedback remain to be measured in later phases.

The Phase 2 simulator and real iPhone cadence, Tailscale/Wi-Fi RTT and loss,
control observations, and foreground/background timing are recorded in
[`phase2_report.md`](phase2_report.md). Thermal behavior remains the explicit
physical-device performance gate.
