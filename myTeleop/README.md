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
rejection reason. Pose quaternion columns use the consistent `qx,qy,qz,qw`
order. Disable this only when needed with `--no-telemetry-log`.

## Safe X5 hardware validation

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

The defaults use XYZ translation with the activation-time tool orientation held,
`0.1` translation scale, a `0.5 rad/s` joint speed limit, and a `0.05 s` SDK
command preview. To add orientation following later, set a small value such as
`--rotation-scale-factor 0.2`. Plain position-only IK remains available with
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
