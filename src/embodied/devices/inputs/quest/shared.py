"""Reuse native Quest routing with a single input owner and per-arm namespaces."""
from .composition import NativeRouting, QuestDevice
from .native_config import write_native_configs
from .sources import NativeIngress


class SharedDevice(QuestDevice):
    @staticmethod
    def commands(configs, prefix):
        return {}  # The workbench pair supervisor owns the single input process.


class Routing(NativeRouting):
    @staticmethod
    def configure(directory, prefix, settings):
        vr = settings.vr
        if vr.input_prefix is None:
            raise ValueError('Shared Quest requires an explicit input owner prefix')
        return write_native_configs(directory, prefix, settings.robot.gripper_max_width,
            settings.sdk_library, vr.base_yaw_deg, vr.grip_threshold, vr.position_scale,
            controller=vr.controller, input_prefix=vr.input_prefix,
            feedback_hz=settings.robot.control_hz,
            cartesian_speed_limits=(settings.robot.teleop_linear_speed_m_s,
                                    settings.robot.teleop_angular_speed_rad_s))


class Ingress(NativeIngress):
    def __init__(self, prefix, dependencies, gripper=True, settings=None):
        super().__init__(prefix, dependencies, gripper)
        from embodied.control.ipc.services import open_service, make_subscriber
        from embodied.control.ipc.types import XrControllerInput
        vr = settings.vr
        self.subscribers['input'] = make_subscriber(open_service(self.node,
            f'{vr.input_prefix}/xr/{vr.controller}_controller/states/input', XrControllerInput))
