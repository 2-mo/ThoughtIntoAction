# Quest Pro + ARX R5 single-arm teleoperation

This directory contains a minimal single-arm entry that reuses the existing
`XRoboToolkit-Teleop-Sample-Python` controller and the ARX R5 SDK.

## Configuration

- Python: `/home/r2/miniconda3/envs/xrobot312`
- Robot: one ARX R5
- SocketCAN: `can0`
- XR input: Quest right controller
- Arm activation: hold the right grip
- Gripper: right trigger
- Camera and logging: disabled by default

The upstream controller moves the arm to its home state once during startup and
again during a normal shutdown. The wrapper prevents the existing duplicate
hardware initialization, but otherwise keeps upstream behavior unchanged.

## Prerequisites

1. Start XRoboToolkit PC Service on the computer.
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
