# `arx5_test` Function Mapping

Initial Phase 0 reference repository: `2-mo/arx5_test` at
`0dbffd38bd4032df80c0913bfabf821e7c8c2175`.

Later Phase 5 local evidence was re-read from
`/home/r2/Desktop/arx5_test` at
`69a144f1cd7b6157a536428096b1f52e25d9200e`; see
[`references.md`](references.md) and [`phase5_report.md`](phase5_report.md).

This document maps relevant behavior, not source code. The reference has no
repository-level license at the inspected revision, so no implementation was
copied or adapted.

| Reference behavior | Phase 0 owner | Phase 0 evidence | Later work |
|---|---|---|---|
| Latest target plus monotonic generation | `runtime/latest_value.py`, `runtime/receiver.py` | overwrite and generation unit tests | UDP receiver will store decoded input through the same object |
| Fixed-period servo | `runtime/servo_loop.py` | threaded Mock-to-Mock integration test | measure under IK and SDK load before considering C++ |
| Skip catch-up and rebase a late deadline | `runtime/servo_loop.py`, `runtime/metrics.py` | cycle timing and deadline-rebase metrics | fault policy will be tuned from hardware measurements |
| Target watchdog | `runtime/watchdog.py` | 50/100/300 ms stage tests | Phase 1 adds network loss and stale packet scenarios |
| Feedback timestamp progress | `runtime/servo_loop.py` | frozen-feedback integration test invokes safe stop | verify the ARX5 SDK timestamp source and resolution |
| Emergency damping on worker failure | `RobotBackend.safe_stop()` port, Mock backend | runtime fault and E-stop integration tests | map to a verified ARX5 stop primitive in Phase 5 |
| Online Ruckig state carry-forward | `Controller` boundary | boundary only; deliberately absent | use upstream Ruckig in Phase 5, not a custom S-curve |
| Cartesian planning and IK | `Controller` and future ARX5 adapter | boundary only; deliberately absent | Pinocchio IK and branch-continuity tests in Phase 5 |
| Singularity speed scaling | robot/controller safety boundary | interface and risk documented | derive from ARX5 Jacobian and validate before hardware motion |
| Common workspace and target-step limits | `core/safety.py` | safety and policy tests | intersect with, never replace, backend hard limits |
| Robot-local hard envelope | each `RobotBackend` | Mock backend rejects step/workspace violations | populate from verified ARX5 limits in Phase 5 |
| Cartesian preview/ghost | no Phase 0 owner | explicitly out of scope | Phase 4 visualization |
| Browser API, cameras, perception, calibration, grasping | none | explicitly rejected | not part of the real-time teleoperation core |

## Ownership differences

`arx5_test` combines several product and hardware concerns around an ARX5
application. This repository retains the proven control-loop ideas but assigns
them to replaceable boundaries:

```text
device adapter -> canonical input -> common pipeline -> controller -> backend
                         |                  |                |
                    latest-only       session/safety    IK/Ruckig later
                                                               |
                                                        ARX5 SDK later
```

The mapping intentionally does not preserve the reference repository's web
kernel, browser lifecycle, camera stack, experiments, or compatibility layers.
It also does not imply that the ARX5 SDK's damping call, timestamps, update
rate, or safety checks have been verified on the user's hardware.
