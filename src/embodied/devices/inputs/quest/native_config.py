"""Generate isolated runtime configurations consumed by the bundled binaries."""
from __future__ import annotations

import json
import math
from pathlib import Path

from embodied.control.ipc.services import validate_prefix
from embodied.control.motion_limits import DEFAULT_PATH, validate_native_router


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
                         position_scale=(1., 1., 1.), *, dual: bool = False, controller: str = 'right',
                         input_prefix: str | None = None, feedback_hz: float = 250.,
                         cartesian_speed_limits=None, input_timeout_ms: int = 100) -> dict[str, Path]:
    """Write xrobot, coordinator, arm and gripper TOML files for one run.

    ``prefix`` must be unique per concurrent run and start with ``arx5-``.
    A zero gripper width omits that route. Base yaw rotates the XR reference
    into the robot's native base frame; the robot model itself is not rotated.
    Native coordinator and routers retain the original input/control policy.
    """
    validate_prefix(prefix)
    if controller not in ('left', 'right'):
        raise ValueError('controller must be left or right')
    if not math.isfinite(feedback_hz) or feedback_hz <= 0:
        raise ValueError('feedback_hz must be finite and positive')
    if type(input_timeout_ms) is not int or not 20 <= input_timeout_ms <= 1000:
        raise ValueError('input_timeout_ms must be an integer in 20..1000')
    if input_prefix is not None:
        validate_prefix(input_prefix)
    motion = validate_native_router(json.loads(DEFAULT_PATH.read_text())['native_router'])
    if cartesian_speed_limits is not None:
        for kind, value in zip(('linear', 'angular'), cartesian_speed_limits, strict=True):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Cartesian speeds must be finite positive values')
            key = 'max_' + kind + '_velocity'
            motion['cartesian'][key] = min(motion['cartesian'][key], value)
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
    # A single arm follows the configured hand; a pair uses both controllers.
    chosen = "right" if dual else controller
    xr_root, robot_root = f"{input_prefix or prefix}/xr", f"{prefix}/robot"
    right_inp = f"{xr_root}/right_controller/states/input"
    # The coordinator disables the group on one invalid Episode sample; the
    # relay drops the invalid startup frame before it reaches that topic.
    left_inp = f"{xr_root}/left_controller/states/input_filtered"
    # Only the ARX dual profile publishes a filtered left stream (a relay
    # smooths that hand). A single-arm kernel on either hand reads the raw one.
    chosen_inp = f"{xr_root}/{chosen}_controller/states/input"
    group = f"{prefix}/group"
    permit = f"{group}/permit"

    def controller_channel(name):
        return dict(channel_type=f"{name}_controller", channel_label=f"PICO {name.title()} Controller",
                    kind="robot", enabled=True, mode="free-drive", dof=6,
                    control_frequency_hz=250.0,
                    publish_states=["pose", "input"], recorded_states=[])

    controller_channels = ([controller_channel("right"), controller_channel("left")]
                           if dual else [controller_channel(chosen)])
    xrobot = dict(name="xrobot", driver="xrobot", id=prefix, bus_root=xr_root,
                  sdk_library=str(sdk_library), xr_basis=basis, channels=controller_channels)

    # Dual arms: left controller drives arm1 (can0), right controller drives arm2 (can1).
    arm_specs = [("arm", "gripper", left_inp if dual else chosen_inp,
                  f"{xr_root}/left_controller/states/pose_filtered" if dual
                  else f"{xr_root}/{chosen}_controller/states/pose")]
    if dual:
        arm_specs.append(("arm2", "gripper2", right_inp, f"{xr_root}/right_controller/states/pose"))

    members = []
    routes = []
    common_base = dict(ruckig_smoothing_enabled=motion['smoothing_enabled'],
                       control_rate_hz=250.0, control_group_id=group,
                       relative_hold_timeout_ms=int(max(30, 2000 / feedback_hz)),
                       relative_fault_timeout_ms=int(max(100, 3000 / feedback_hz)))
    router_configs = {}
    for index, (arm_channel, grip_channel, input_topic, pose_topic) in enumerate(arm_specs):
        gate = f"{group}/{arm_channel}/gate"
        parallel = f"{group}/{grip_channel}/parallel"
        arm_health = f"{group}/{arm_channel}/health"
        grip_health = f"{group}/{grip_channel}/health"
        robot_arm = f"{robot_root}/{arm_channel}"
        robot_grip = f"{robot_root}/{grip_channel}"
        members.extend([
            dict(member_id=arm_channel, info_mode_topic=f"{robot_arm}/info/mode",
                 control_mode_topic=f"{robot_arm}/control/mode", disabled_settle_ms=3000),
            dict(member_id=grip_channel, info_mode_topic=f"{robot_grip}/info/mode",
                 control_mode_topic=f"{robot_grip}/control/mode", disabled_settle_ms=3000),
        ])
        routes.extend([
            dict(route_id=arm_channel, input_kind="grip", controller_input_topic=input_topic,
                 grip_threshold=grip_threshold, gate_topic=gate, health_topic=arm_health),
            dict(route_id=grip_channel, input_kind="trigger", controller_input_topic=input_topic,
                 trigger_max_width_m=float(gripper_width), parallel_topic=parallel, health_topic=grip_health),
        ])
        leader = "xr.left_controller" if dual and index == 0 else f"xr.{chosen}_controller"
        router_configs[arm_channel] = dict(
            common_base, process_id=f"{prefix}.{arm_channel}", leader_channel_id=leader,
            follower_channel_id=f"robot.{arm_channel}", leader_state_kind="pose", leader_state_topic=pose_topic,
            follower_command_kind="end_pose", follower_command_topic=f"{robot_arm}/commands/end_pose",
            follower_state_kind="end_effector_pose", follower_state_topic=f"{robot_arm}/states/end_effector_pose",
            follower_mode_info_topic=f"{robot_arm}/info/mode", route_health_topic=arm_health,
            mapping="cartesian", cartesian_relative=True, cartesian_orientation_frame="base",
            cartesian_position_scales=list(position_scale), leader_gate_topic=gate,
            follower_motion_limits=dict(cartesian=dict(
                **motion['cartesian'], max_linear_tracking_error=.05, max_angular_tracking_error=.1)))
        router_configs[grip_channel] = dict(
            common_base, process_id=f"{prefix}.{grip_channel}", leader_channel_id=leader,
            follower_channel_id=f"robot.{grip_channel}", leader_state_kind="parallel_position",
            leader_state_topic=parallel, follower_command_kind="parallel_mit",
            follower_command_topic=f"{robot_grip}/commands/parallel_mit",
            follower_state_kind="parallel_position", follower_state_topic=f"{robot_grip}/states/parallel_position",
            follower_mode_info_topic=f"{robot_grip}/info/mode", route_health_topic=grip_health,
            mapping="parallel", group_permit_topic=permit, group_permit_timeout_ms=100,
            follower_motion_limits=dict({key: [value] for key, value in motion['gripper'].items()},
                                        max_tracking_error=[.02]),
            command_defaults=dict(parallel_mit_kp=[1.0], parallel_mit_kd=[0.0]))

    coordinator = dict(
        process_id=f"{prefix}.coordinator", group_id=group,
        mode_controller_input_topic=(right_inp if dual else chosen_inp),
        episode_controller_input_topic=left_inp if dual else f"{prefix}/unused-recording-input",
        permit_topic=permit, button_chord_ms=150.0, input_timeout_ms=input_timeout_ms,
        # The vendor uses mode_ack_timeout as the initial-readiness window; a
        # dual-arm device takes longer to publish both controller streams.
        # A shared owner covers a second controller that is still waking up.
        mode_ack_timeout_ms=20000 if dual or input_prefix is not None else 1000,
        route_health_timeout_ms=500,
        members=members, routes=routes)
    configs = dict(xrobot=xrobot, coordinator=coordinator, **router_configs)
    if gripper_width == 0:
        coordinator['members'] = [member for member in members if not member['member_id'].startswith('gripper')]
        coordinator['routes'] = [route for route in routes if route['input_kind'] != 'trigger']
        for name in [name for name in configs if name.startswith('gripper')]:
            del configs[name]
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name, data in configs.items():
        path = directory / f"{name}.toml"
        path.write_text(_toml(data), encoding="utf-8")
        paths[name] = path
    return paths
