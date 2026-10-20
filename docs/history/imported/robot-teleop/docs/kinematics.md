# Offline Kinematics

Phase 4 contains a deliberately isolated model preflight. It uses the optional
`kinematics` extra (`pin==4.1.0`) and the source-attributed X5 URDF under
`assets/robots/arx5/`.

Run it with:

```bash
make phase4-kinematics-check
```

The tool loads `configs/robots/arx5.yaml`, validates that
`hardware_enabled: false`, builds Pinocchio model data, and writes
`build/phase4_kinematics_check.json`. Its deterministic input sequence covers
Clutch reference capture, relative translation, Recenter continuity, and three
small relative rotations. Each Cartesian target is solved from the prior joint
solution; the report records residuals, joint-space steps, branch continuity,
and Jacobian singular values.

The X5 URDF is a geometric model. The position and velocity limits in the
configuration are cited from the matching ARX5 SDK revision, but they are not a
claim about the installed arm. The model seed is synthetic and must never be
interpreted as a home command. The report is only a feasibility signal: it does
not cover the real mounting transform, payload, collision environment,
calibration, physical joint offsets, firmware behavior, CAN state, or SDK stop
semantics.

`kinematics/` is outside `core/` and is never imported by Receiver or Servo.
It does not import `arx5-interface`, open a socket, create CAN traffic, or move
hardware. A future `Arx5Backend` must still independently verify hardware
identity and intersect live targets with backend limits.
