# Hardware Checklist

Phases 0 through 5 do not authorize robot motion. This checklist is a gate for
the operator-only Phase 6 and does not itself authorize execution.

- [ ] Phase 0 through Phase 5 tests and reviews complete.
- [ ] Correct robot model and CAN interface confirmed by operator.
- [ ] Arm securely mounted; workspace cleared.
- [ ] Physical emergency stop present, reachable, and tested.
- [ ] Operator remains next to the arm.
- [ ] `hardware.enabled` remains false in default profiles.
- [ ] Explicit `--enable-hardware` required at invocation.
- [x] State-only connection validated before command mode; the five-sample
  operator run had advancing SDK and host timestamps with no framework writes.
- [x] Backend `safe_stop` tested before motion; the gated SDK damping run on
  `can0` returned `safe_stop_passed=true` with no Servo, Cartesian,
  joint-position, or gripper command.
- [ ] Network-loss, tracking-loss, and clutch-release stops tested.
- [ ] First movement is a low-speed single-axis test.
- [ ] Position tests precede orientation and gripper tests.
- [ ] Logs and latency report reviewed before increasing speed.
