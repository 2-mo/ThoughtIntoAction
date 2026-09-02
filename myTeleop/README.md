# Quest Pro + ARX X5 single-arm teleoperation

This directory contains a minimal single-arm entry that reuses the existing
`XRoboToolkit-Teleop-Sample-Python` controller and the maintained ARX5 SDK.

## Configuration

- Python: `/home/r2/miniconda3/envs/xrobot312`
- Robot: one ARX X5
- SocketCAN: `can0`
- XR input: Quest right controller
- Arm activation: hold the right grip
- Gripper: right trigger
- Camera and logging: disabled by default

The local wrapper uses the maintained `arx5-interface` backend. Teleoperation
starts from the measured arm pose without automatic homing and switches the arm
to damping mode during a normal shutdown.

## Prerequisites

1. For USB transport, connect and authorize the Quest, then run:

```bash
./run.sh setup-quest-usb
```

Set the XRoboToolkit server IP in Quest to `127.0.0.1`, then press Connect.
Start XRoboToolkit PC Service on the computer.
2. Open the XRoboToolkit application in Quest Pro.
3. Confirm the ARX adapter and CAN interface:

   ```bash
   ls -l /dev/arx_r5
   ip -brief link show can0
   ```

If `can0` is absent, recreate it with:

```bash
sudo slcand -o -f -s8 /dev/arx_r5 can0
sudo ip link set can0 up
```

## Checks

Run from this directory:

```bash
./run.sh check-xr
./run.sh check-arm
```

`check-xr` must show a non-zero, changing timestamp. `check-arm` only reads
joint feedback and does not call a motion command.

To isolate the SDK and motor control path from XR and IK, first read the X5:

```bash
./run.sh test-joint --joint 1
```

After supporting the arm and clearing its workspace, explicitly enable a small
`0.03 rad` J1 move-and-return test at `0.05 rad/s`:

```bash
./run.sh test-joint --joint 1 --actuate
```

## Official ARX X5-2025 Python backend

The AC-one / dual-rail-gripper arm is the X5-2025 variant. Its official SDK
selects the matching driver and `x5_2025.urdf` with `type=2`; this is more than
a URDF-only change. Build it once with:

```bash
./run.sh build-x5-2025-sdk
```

No ROS nodes are used. The SDK links against `kdl_parser`, which checks that
`AMENT_PREFIX_PATH` exists; `run.sh` supplies the Conda prefix for that check.

Read-only feedback check (always returns to `SOFT(0)`):

```bash
./run.sh test-joint-2025 --joint 3
```

Small explicit actuation test, only with the arm supported:

```bash
./run.sh test-joint-2025 \
  --joint 3 \
  --delta-rad 0.01 \
  --max-joint-speed-rad-s 0.01 \
  --settle-tolerance-rad 0.002 \
  --timeout-s 5 \
  --actuate
```

The experimental XR entry is read-only:

```bash
./run.sh teleop-x5-2025
```

Actuation through this entry is blocked because isolated tests of the official
closed-source `POSITION_CONTROL(5)` showed uncommanded multi-joint drift. The
SDK remains available for read-only comparison and model reference.

## X5-2025 hybrid dynamics model

`arx5-sdk/models/X5_2025_hybrid.urdf` keeps Stanford's limits, collision
geometry and `eef_link`, while using the official ARX X5-2025 inertial
properties. Both `simulate-official-x5` and `teleop-official-x5` use this same
model by default. The hardware path remains Stanford's inspectable joint
controller; only its inverse-dynamics model has changed.

The test holds the measured gripper position, returns to its initial joint
angle on normal completion, and always switches the arm to damping on exit.
Joint feedback is copied while its native SDK state object remains alive and is
validated before command generation; corrupt/non-finite feedback blocks the
command and immediately requests damping.
The motion test also stops on excessive non-commanded joint drift, test-joint
overshoot, or abnormal measured joint speed.

To inspect the Quest right controller in a read-only browser view without
loading the arm SDK or accessing CAN, run:

```bash
./run.sh visualize-xr
```

Then open `http://127.0.0.1:8765`. The page shows three orthographic views,
the controller trail, stream freshness, grip, and trigger values.

To drive the ARX X5 3D model through the same Placo IK without loading the arm
SDK or accessing CAN, run:

```bash
./run.sh simulate-x5
```

Hold the right grip to move the virtual arm. The Meshcat page opens
automatically in the browser.

Simulation and hardware use the same XR validation, Grip clutch, IK, joint-limit
checks, joint-rate limiter, and command selection code. Only the final backend
is different: simulation updates an in-memory X5, while hardware writes through
`arx5-interface` to `can0`.

## Telemetry logs

Simulation, dry-run, and hardware teleoperation create one CSV per session in:

```text
myTeleop/logs/x5_<source>_<timestamp>.csv
```

Each row contains the raw right-controller pose and timestamp, Grip/Trigger,
actual X5 joint positions, IK joint target, rate-limited command, actual and IK
end-effector poses, gripper state, maximum joint error, command status, and any
rejection reason. The exact velocity and torque fields submitted to the SDK are
also recorded; normal position commands submit zeros so gravity compensation is
applied exactly once inside the SDK. Pose quaternion columns use the consistent
`qx,qy,qz,qw` order. Disable this only when needed with `--no-telemetry-log`.

## Safe X5 hardware validation

To establish a clean baseline, run XRoboToolkit's stock ARX Grip/IK/control
pipeline with only the device model and X5 SDK backend adapted. It defaults to
read-only/damping mode and prints official IK targets without sending them:

```bash
./run.sh simulate-official-x5
./run.sh teleop-official-x5
```

Both baseline adapters use the X5 SDK joint limits and a `0.2 rad/s` default
joint-rate limit. The official simulation advances its feedback at that finite
rate instead of teleporting to each IK target, so physical tracking lag and IK
target changes remain visible during validation.

Do not add `--actuate` until the read-only targets have been checked. The stock
ARX controller's automatic homing and recursive shutdown are intentionally
disabled by this wrapper; neither is needed to compare the control pipeline.

Compute Quest-to-X5 IK against live joint feedback without sending position or
gripper commands. The SDK remains in damping mode:

```bash
./run.sh teleop-dry-run
```

When `active=True` and the reported `max_error` stays small, start bounded XYZ
teleoperation:

```bash
./run.sh teleop
```

The defaults use `0.1` translation scale, `0.5` orientation scale, a `0.5 rad/s`
joint speed limit, and a `0.05 s` SDK command preview. To hold the activation-time
tool orientation, set `--rotation-scale-factor 0`. Plain position-only IK remains available with
`--control-mode position`, but it leaves the wrist orientation unconstrained.

## Run teleoperation

```bash
./run.sh teleop
```

Hold the Quest right grip to activate arm following. Use the right trigger for
the gripper. Press `Ctrl+C` in the terminal to stop.

Available options:

```bash
./run.sh teleop --help
```

For example, enable Placo visualization with:

```bash
./run.sh teleop --visualize-placo
```
