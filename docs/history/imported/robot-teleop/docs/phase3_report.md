# Phase 3 Mock Validation Report

Date: 2026-08-04

Status: in progress. The automated Mock boundary is implemented and verified;
connected-iPhone and visual coordinate review remain operator gates. No real
robot backend, ARX5 SDK, CAN path, IK, Ruckig, or hardware command is present.

## Delivered boundary

| Area | Result |
|---|---|
| Protocol | v1.1 adds fixed 36-byte `EStopResetRequest` and `EStopResetAck` packets while decoding v1.0 messages |
| Golden vectors | Swift and Python share Pose, Ping, Pong, Settings, SettingsAck, reset request, and reset ACK binaries |
| Ubuntu reset gate | Request plus later released, normal-tracking Pose becomes ready, but cannot reset automatically |
| Local authority | `TeleopApplication.reset_estop()` is the sole code path that re-arms MockRobot and sends the ACK |
| iPhone state | Reset request keeps the UI locked until a matching source/session/sequence/timestamp ACK arrives |
| Regression scenario | Real loopback UDP exercises mapping, Clutch/Recenter, tracking, sessions, invalid inputs, burst disposal, reset, and Watchdog |
| Operator console | `make phase3-mock-session` exposes `status`, `report`, `reset`, and `quit` for the Mock-only composition |

The reset request never re-arms Ubuntu by itself. A safe release proves that
the phone is no longer requesting E-stop, then a local operator call completes
the Mock-only recovery. Duplicate completed requests re-send their matching ACK
without a second re-arm. The implementation does not describe, model, or
replace a physical emergency stop.

## Automated evidence

The deterministic validation command is:

```bash
make phase3-mock-validation
```

It creates an ephemeral `127.0.0.1` UDP endpoint and reports
`"hardware_enabled": false`. The checked scenario asserts:

- signed-axis mapping at position scale `0.5` and rotation scale `0.5`, with
  an independent expected orientation axis;
- no target jump on Clutch release/reactivation, Recenter, tracking recovery,
  or a new session;
- malformed, duplicate, out-of-order, retired-session, and input-jump
  rejection;
- bounded UDP batches and latest-only delivery ending at the final burst
  sequence;
- no E-stop ACK before the explicit local reset call;
- startup remains stationary through a release-only baseline until active
  control arms the Watchdog, while a later input loss still reaches the
  configured Watchdog safe-stop;
- HOLD, BRAKE, and SAFE_STOP Watchdog stages after input loss;
- bounded Mock command history and clean Receiver, Servo, and metrics shutdown.

The normal CI-sized scenario completes in about one second. It has dedicated
coverage in `tests/integration/test_phase3_mock_validation.py`, including the
operator console refusing a reset when no E-stop recovery is ready.

## Long Soak

The completed 10-minute loopback soak used:

```bash
uv run python tools/phase3_mock_validation.py --duration 600
```

Its final JSON is retained in ignored `build/` output. The local result was:

| Measurement | Result |
|---|---:|
| Requested stream | 600 s at 60 Hz, 36,000 Pose packets |
| Accepted inputs / Servo cycles | 36,066 / 120,026 |
| Effective Servo frequency | 198.92 Hz |
| P95 compute / jitter | 0.445 / 1.254 ms |
| P99 compute / jitter | 0.879 / 4.558 ms |
| Deadline misses / rebases | 739 / 30 |
| Maximum consecutive deadline misses | 2 |
| Local loopback RTT | 0.122 ms |
| Packet age P95 / P99 | 18.813 / 302.135 ms |
| Latest-only superseded datagrams | 79 |
| Command history | 256 / 256 bounded entries |
| Mock safe stops | 2 expected events: E-stop, then Watchdog |
| Worker shutdown | all stopped |

The final input pause deliberately contributes the high packet-age tail and the
HOLD/BRAKE/SAFE_STOP counts. The 739 deadline misses are a measurable macOS
scheduling result, not a hidden pass condition; no fault occurred and no burst
of more than two consecutive misses was observed. This must be remeasured on
the Ubuntu control host before deciding whether real-time-path optimization is
needed. This is a local Mock stability test only. Its RTT is neither a
Tailscale result nor an estimate of iPhone-to-Ubuntu one-way latency.

## Ubuntu Preflight

The clean Ubuntu checkout at `r2@100.112.78.10` was switched to this feature
branch and ran the same Mock-only validator for two seconds. It completed with
all scenario checks true, 196.51 Hz effective Servo frequency, zero deadline
misses, a 3.32 ms Clutch-release-to-HOLD observation, and every worker stopped.
No robot backend or CAN process was present.

`uv` is installed at `/home/r2/.local/bin/uv`, but that directory is absent
from a non-interactive SSH PATH. The preflight used the explicit path; it did
not change shell startup files. Use the same path on Ubuntu until the operator
chooses to configure their own PATH.

## Connected-iPhone Reset Evidence

On 2026-08-04, the operator repeated the two-sided E-stop recovery against the
Ubuntu Mock console using the physical iPhone. After the console update that
keeps startup `DISCONNECTED` until the first input, the phone's reset request
became ready and the explicit local `reset` completed the handshake. The iOS UI
unlocked normally only after the matching acknowledgement. This confirms the
Mock-only protocol path; it is not a physical emergency-stop validation.

## Connected-iPhone Timing Finding

An initial connected-iPhone Mock run exposed one `Cartesian target velocity
exceeds the limit` fault. It was not a Watchdog safe-stop: the observed stream
had a small estimated sequence loss, three BRAKE cycles, and no Watchdog final
stop before the Servo faulted. The Receiver continued accepting later Poses
after the Servo exited, so its post-fault packet counters are not uninterrupted
control-loop evidence.

The fault revealed two issues: the prior implementation evaluated a 60 Hz input
target change with the 200 Hz Servo period, and a genuinely fast but continuous
hand motion was treated as a latched fault. The corrected path uses consecutive
Ubuntu input receipt times and latest-only Cartesian slew limiting. It does not
queue old targets; hard input jumps, non-finite values, and workspace violations
remain rejected. The connected-iPhone ten-minute sample remains an explicit
rerun gate after this correction.

## Post-fix Connected-iPhone Smoke Sample

On 2026-08-04, the operator ran the updated branch through the physical iPhone
and Ubuntu Mock console for a short approximately 24-second sample. The sample
accepted 1,393 Poses and completed 4,835 Servo cycles. It produced no fault,
no Servo error, no Mock safe stop, and all three runtime workers remained
running. The Servo frequency was 200.000 Hz; P95/P99 compute time was
0.205/0.210 ms. Input inter-arrival median/P99 was 16.686/31.544 ms, while
packet-age median/P95/P99 was 8.219/18.501/25.458 ms (maximum 41.700 ms).
There was one estimated sequence loss in 1,406 Pose packets and 13 isolated
deadline misses, with no consecutive misses.

One 745.827 ms input-arrival gap was observed. The final state was `HOLD`, and
the report contained no HOLD/BRAKE/SAFE_STOP Watchdog samples, so this gap is
treated as an inactive-control or reconnect observation rather than an active
control latency result. A longer connected run with Clutch held continuously is
deferred to the pre-hardware confidence gate rather than blocking Phase 4
Mock/Ghost visualization work.

Phase 3 reports now retain 131,072 samples, enough for 10 minutes and 55
seconds at 200 Hz. Their `sample_window` field explicitly distinguishes
whole-session percentile data from a recent bounded window, avoiding an
incorrect comparison between cumulative Watchdog counters and truncated
latency samples.

## Quality Gate

Before finalizing this report, the branch has passed:

| Check | Result |
|---|---|
| Ruff check and format | pass |
| Strict mypy | pass, 60 source files |
| Python test suite | pass, 184 tests |
| Phase 3 deterministic scenario | pass |
| Mock console scripted lifecycle | pass; reset without ready E-stop rejected |
| iOS Simulator XCTest | pass, 36 tests, 2 existing conditional skips |
| iOS Release builds | pass for generic simulator and generic physical iOS targets, unsigned |

## Remaining Operator Gates

- Review position signs, handedness, orientation axes, scale, Clutch,
  Recenter, tracking loss/recovery, session restart, and network recovery using
  Mock/Ghost only.
- Run a longer connected-iPhone active-control stability sample before hardware
  backend work. The current local soak cannot answer thermal, Wi-Fi/Tailscale,
  or ARKit source-cadence questions.
- Record the deferred iPhone thermal/battery and final portrait/landscape
  ergonomics observations from Phase 2.

Phase 3 authorizes Phase 4 Mock/Ghost visualization work only. It does not
authorize Phase 5 ARX5 work or any physical robot motion.
