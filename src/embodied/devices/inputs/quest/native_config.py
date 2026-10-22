"""Generate isolated runtime configurations consumed by the bundled binaries."""
from __future__ import annotations

import json
import math
from pathlib import Path

from embodied.control.ipc.services import validate_prefix


def _toml(data: dict, section: str = "") -> str:
    """Serialize the small, fixed runtime schema without another dependency."""
    lines = []
    for key, value in data.items():
        if not isinstance(value, dict) and not (isinstance(value, list) and value and isinstance(value[0], dict)):
            lines.append(f"{key} = {json.dumps(value, ensure_ascii=False)}")
    for key, value in data.items():
        full = f"{section}.{key}" if section else key
        if isinstance(value, dict):
            lines.extend([f"\n[{full}]", _toml(value, full)])
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            for item in value:
                lines.extend([f"\n[[{full}]]", _toml(item, full)])
    return "\n".join(lines) + "\n"


def write_native_configs(directory: Path, prefix: str, gripper_width: float, sdk_library: str,
                         base_yaw_deg: float = 0., grip_threshold: float = .7,
                         position_scale=(1., 1., 1.)) -> dict[str, Path]:
    """Write xrobot, coordinator, arm and gripper TOML files for one run.

    ``prefix`` must be unique per concurrent run and start with ``arx5-``.
    A zero gripper width omits that route. Base yaw rotates the XR reference
    into the robot's native base frame; the robot model itself is not rotated.
    Native coordinator and routers retain the original input/control policy.
    """
    validate_prefix(prefix)
    if not math.isfinite(gripper_width) or gripper_width < 0:
        raise ValueError("gripper_width must be nonnegative and finite")
    if not math.isfinite(base_yaw_deg):
        raise ValueError('base_yaw_deg must be finite')
    if not math.isfinite(grip_threshold) or not 0 < grip_threshold <= 1:
        raise ValueError('grip_threshold must be in (0, 1]')
    if len(position_scale) != 3 or not all(math.isfinite(x) and x > 0 for x in position_scale):
        raise ValueError('position_scale must contain three finite positive values')
    angle = math.radians(-base_yaw_deg)
    c, s = math.cos(angle), math.sin(angle)
    # Rz(-base_yaw) @ the existing row-major XR-to-X5 basis.
    basis = [s, 0., -c, -c, 0., -s, 0., 1., 0.]
    basis = [0. if abs(value) < 1e-12 else value for value in basis]
    xr_root, robot_root = f"{prefix}/xr", f"{prefix}/robot"
    inp = f"{xr_root}/right_controller/states/input"
    group = f"{prefix}/group"
    permit = f"{group}/permit"
    gate, parallel = f"{group}/arm/gate", f"{group}/gripper/parallel"
    arm_health, grip_health = f"{group}/arm/health", f"{group}/gripper/health"
    xrobot = dict(name="xrobot", driver="xrobot", id=prefix, bus_root=xr_root,
                  sdk_library=str(sdk_library),
                  xr_basis=basis,
                  channels=[dict(channel_type="right_controller", channel_label="PICO Right Controller",
                                 kind="robot", enabled=True, mode="free-drive", dof=6,
                                 control_frequency_hz=250.0, publish_states=["pose", "input"], recorded_states=[])])
    coordinator = dict(
        process_id=f"{prefix}.coordinator", group_id=group,
        mode_controller_input_topic=inp,
        episode_controller_input_topic=f"{prefix}/unused-recording-input",
        permit_topic=permit, button_chord_ms=150.0, input_timeout_ms=100,
        mode_ack_timeout_ms=1000, route_health_timeout_ms=500,
        members=[dict(member_id=channel, info_mode_topic=f"{robot_root}/{channel}/info/mode",
                      control_mode_topic=f"{robot_root}/{channel}/control/mode", disabled_settle_ms=3000)
                 for channel in ("arm", "gripper")],
        routes=[dict(route_id="arm", input_kind="grip", controller_input_topic=inp,
                     grip_threshold=grip_threshold, gate_topic=gate, health_topic=arm_health),
                dict(route_id="gripper", input_kind="trigger", controller_input_topic=inp,
                     trigger_max_width_m=float(gripper_width), parallel_topic=parallel, health_topic=grip_health)])
    common = dict(leader_channel_id="xr.right_controller", ruckig_smoothing_enabled=False,
                  control_rate_hz=250.0, control_group_id=group,
                  relative_hold_timeout_ms=30, relative_fault_timeout_ms=100)
    arm = dict(common, process_id=f"{prefix}.arm", follower_channel_id="robot.arm",
               leader_state_kind="pose", leader_state_topic=f"{xr_root}/right_controller/states/pose",
               follower_command_kind="end_pose", follower_command_topic=f"{robot_root}/arm/commands/end_pose",
               follower_state_kind="end_effector_pose", follower_state_topic=f"{robot_root}/arm/states/end_effector_pose",
               follower_mode_info_topic=f"{robot_root}/arm/info/mode", route_health_topic=arm_health,
               mapping="cartesian", cartesian_relative=True, cartesian_orientation_frame="base",
               cartesian_position_scales=list(position_scale), leader_gate_topic=gate,
               follower_motion_limits=dict(cartesian=dict(
                   max_linear_velocity=1.0, max_linear_acceleration=10.0, max_linear_jerk=100.0,
                   max_angular_velocity=2.0, max_angular_acceleration=20.0, max_angular_jerk=200.0,
                   max_linear_tracking_error=.05, max_angular_tracking_error=.1)))
    gripper = dict(common, process_id=f"{prefix}.gripper", follower_channel_id="robot.gripper",
                   leader_state_kind="parallel_position", leader_state_topic=parallel,
                   follower_command_kind="parallel_mit", follower_command_topic=f"{robot_root}/gripper/commands/parallel_mit",
                   follower_state_kind="parallel_position", follower_state_topic=f"{robot_root}/gripper/states/parallel_position",
                   follower_mode_info_topic=f"{robot_root}/gripper/info/mode", route_health_topic=grip_health,
                   mapping="parallel", group_permit_topic=permit, group_permit_timeout_ms=100,
                   follower_motion_limits=dict(max_velocity=[.5], max_acceleration=[5.0], max_jerk=[50.0], max_tracking_error=[.02]),
                   command_defaults=dict(parallel_mit_kp=[1.0], parallel_mit_kd=[0.0]))
    configs = dict(xrobot=xrobot, coordinator=coordinator, arm=arm, gripper=gripper)
    if gripper_width == 0:
        coordinator['members'] = coordinator['members'][:1]
        coordinator['routes'] = coordinator['routes'][:1]
        del configs['gripper']
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name, data in configs.items():
        path = directory / f"{name}.toml"
        path.write_text(_toml(data), encoding="utf-8")
        paths[name] = path
    return paths
