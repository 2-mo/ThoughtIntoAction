# Extension Guide

## Add an input device

1. Decode device data outside `core/`.
2. Convert units, axes, and quaternion order to canonical conventions.
3. Implement `TeleopInputSource` and truthful `InputCapabilities`.
4. Reject non-finite data before publishing `TeleopInput`.
5. Add the class explicitly to `INPUTS`.
6. Run the shared input contract and integration matrix.

For a UDP source, keep byte transport, generic packet codec, and device decoder
separate. Assign a stable `SourceType`, declare capabilities at the adapter
boundary, and add cross-language golden vectors before another client ships.
Do not add a source-name branch to canonical behavior.

Do not add IK, robot limits, SDK calls, workspace logic, clutch state, or a new
watchdog to the adapter.

## Add a protocol message

1. Change the major version for incompatible layout or semantic changes.
2. Keep a fixed little-endian `Struct` and exact declared size.
3. Validate the common header before message-specific payload fields.
4. Add deterministic binary and JSON golden fixtures.
5. Run the same fixtures in every sender language.
6. Keep Pose unacknowledged and avoid any historical-command queue.

Do not overload packet size as message identity or put robot semantics in the
transport.

## Add a robot backend

1. Keep SDK imports inside the backend module.
2. Implement idempotent connect/disconnect, canonical state, command validation,
   hold, and safe stop.
3. Convert SDK units and quaternion ordering exactly once at this boundary.
4. Enforce hard robot limits independently of common safety.
5. Require the explicit hardware gate for any real backend.
6. Add the class explicitly to `ROBOTS` and run the backend contract.

Do not import input-device protocols or duplicate clutch/mapping/filtering.

The current ARX5 foundation adds one more gate: an enabled backend must receive
an explicit interface name, and its SDK loader remains lazy. Do not construct a
vendor controller from a module import, test fixture, or default profile.

## Add a control policy

1. Implement `ControlPolicy.reset()` and `update()`.
2. Depend on canonical models and capabilities, never `source_name`.
3. Make reset produce a continuous first target.
4. Return HOLD for invalid/inactive input if called defensively.
5. Add the class explicitly to `POLICIES` and run the policy contract.

## Add compliance

Implement `CompliancePolicy.modify(target, state, dt)`. The default remains
`NoCompliancePolicy`. A future local admittance layer is permitted only after
position servo, feedback, force estimation, limits, and contact testing are
verified. Network-supplied joint torque is not an extension path.

## Architecture check

Run:

```bash
uv run pytest tests/test_architecture.py
uv run mypy
```

A component is not accepted if replacing it requires edits to core behavior or
an unrelated adapter.
