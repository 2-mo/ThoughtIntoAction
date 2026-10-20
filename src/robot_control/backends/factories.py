"""Backend construction at the application boundary; imports stay local to each factory."""


def service(*, settings, initial_q=None):
    if initial_q is not None:
        raise ValueError("Set the initial virtual pose on arm-service, not on a control client")
    from robot_core.model import RobotModel
    from robot_control.client import ArmClient
    return ArmClient(RobotModel(settings["dynamics_urdf_path"]),
                     path=settings.get("service_socket"), hardware=settings["hardware"],
                     actuate=settings["actuate"], input_name=settings["input_name"],
                     speed=settings["max_joint_speed_rad_s"],
                     motion={**{key: settings[key] for key in ("max_command_lead_rad", "preview_time_s", "joint_soft_limit_margin_rad")}, **settings.get("trajectory_settings", {})})


def stanford(*, settings, initial_q=None):
    if initial_q is not None:
        raise ValueError("initial_q is a simulation setting, not a real-arm command")
    from robot_control.hardware import OfficialX5HardwareAdapter
    from robot_control.online_motion import DEFAULTS
    return OfficialX5HardwareAdapter(**{k: v for k, v in settings.items() if k not in DEFAULTS})


def simulated(*, settings, initial_q=None):
    from robot_core.model import RobotModel
    from robot_control.backends.simulated import SimulatedX5
    return SimulatedX5(
        RobotModel(settings["dynamics_urdf_path"]), initial_q=initial_q,
        gripper_width=settings["gripper_width"], control_rate_hz=settings["control_rate_hz"],
        max_joint_speed_rad_s=settings["max_joint_speed_rad_s"],
        joint_soft_limit_margin_rad=settings["joint_soft_limit_margin_rad"],
    )


def arx2025(*, settings, initial_q=None):
    if settings["actuate"]:
        raise ValueError("Official ARX position actuation remains disabled; see docs/setup.md")
    if initial_q is not None:
        raise ValueError("initial_q is only available for simulation")
    from robot_control.backends.arx2025 import ARXX52025HardwareAdapter
    return ARXX52025HardwareAdapter(**settings)
