"""Reuse the iOS wire codec, session checks and relative mapping with the arx gateway."""
import argparse
from dataclasses import replace
import json
import hashlib
from pathlib import Path
import time

from robot_teleop.inputs.arkit import ArkitInputSource
from robot_teleop.core.mapping import PoseMapping, AxisMapping, MappingOutputFrame, MappingScales
from robot_teleop.core.models import Pose, Quaternion, Vector3, RobotState, TrackingQuality
from robot_teleop.policies.relative_pose import RelativePosePolicy
from robot_teleop.transports.udp import UdpTransport, UdpTransportConfig
from runtime_client.control import ControlClient, ControlSession


class PhoneInput(ArkitInputSource):
    """Latch discrete controls before the upstream batch chooses the newest pose."""
    def __init__(self, transport, on_controls, **options):
        super().__init__(transport, **options)
        self.on_controls = on_controls

    def _handle_payload(self, payload, received_at_ns):
        value = super()._handle_payload(payload, received_at_ns)
        if value is not None:
            self.on_controls(value)
        return value


def phone_tool():
    return PoseMapping(AxisMapping((1, 0, 2), (1, -1, 1)), 2., 1., MappingOutputFrame.TOOL)


def phone_base():
    return replace(phone_tool(), output_frame=MappingOutputFrame.BASE)


class PhoneController:
    def __init__(self, directory, bind_host='127.0.0.1', port=5005, mapping='phone_tool', ghost=False):
        from component_api.registry import Registry
        registry = Registry()
        self.mapping = registry.create('mapping', mapping)
        self.policy = RelativePosePolicy(self.mapping, mapping_provider=lambda: self.mapping)
        self.ghost = ghost
        self.mapping_name = mapping
        source = Path(__file__).resolve().parents[1]
        source_hashes = {str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in [Path(__file__).resolve(), *sorted((source / 'robot_teleop').rglob('*.py'))]}
        self.client = ControlClient(directory) if ghost else ControlSession(directory, source='phone', purpose='teleop',
            metadata={'plugin': 'phone.arkit', 'protocol': 1, 'mapping': mapping, 'position_scale': 2.,
                      'rotation_scale': 1., 'pose_frame': 'base_tcp', 'quaternion': 'xyzw', 'gripper': 'metres',
                      'source_sha256': source_hashes})
        self.transport = UdpTransport(UdpTransportConfig(bind_host=bind_host, bind_port=port, receive_timeout_s=.02))
        self.source = registry.create('input', 'arkit', transport=self.transport, on_controls=self._controls,
            settings_callback=self._settings, estop_reset_request_callback=lambda _: False)
        self.released = self.active = False
        self.session_id = None
        self.last_received = 0
        self.estop = self.home = False
        self.home_pressed = False
        self.recenter = self.recenter_pressed = False
        self.zero_requested = False
        self.operation_id = None
        self.last_target = None

    def _settings(self, scales):
        # Preserve the upstream policy's latch-at-reference semantics.
        accepted = MappingScales(min(scales.position_scale, 3.), scales.rotation_scale)
        self.mapping = self.mapping.with_scales(accepted)
        if not self.ghost:
            self.client.command('source-metadata', metadata={'mapping': self.mapping_name,
                'position_scale': accepted.position_scale, 'rotation_scale': accepted.rotation_scale,
                'activation': 'next clutch/reference'})
        return accepted

    def _controls(self, value):
        self.estop |= value.estop_requested
        if value.home_requested and not self.home_pressed:
            self.home = True
        self.home_pressed = value.home_requested
        self.recenter |= value.recenter_requested and not self.recenter_pressed
        self.recenter_pressed = value.recenter_requested
        if value.joint_zero_requested:
            self.zero_requested = True

    def start(self):
        if not self.ghost:
            self.client.__enter__()
        try:
            self.source.start()
        except BaseException:
            if not self.ghost:
                self.client.close()
            raise

    def _hold(self):
        if self.active and not self.ghost:
            self.client.command('hold')
        self.active = False

    def latest(self):
        value = self.source.latest()
        now = time.monotonic()
        if self.estop:
            if not self.ghost:
                self.client.request('stop')
            raise RuntimeError('Phone ESTOP latched; restart the runtime before re-enabling')
        if self.zero_requested:
            raise ValueError('Motor zero calibration is not a phone control action; use the explicit calibration workflow')
        if value is None:
            if now - self.last_received > .1:
                self._hold()
                self.released = False
            return None
        self.last_received = now
        if value.session_id != self.session_id:
            self._hold()
            self.session_id, self.released = value.session_id, False
        if value.tracking_quality != TrackingQuality.NORMAL:
            self._hold()
            self.released = False
            return None
        snapshot = self.client.status()
        if self.operation_id:
            op = next((op for op in snapshot['operations'] if op['operation_id'] == self.operation_id), None)
            if op and op['status'] != 'running':
                self.operation_id = None
                self.released = self.active = False
            return None
        if self.home:
            self.home = False
            self._hold()
            if not self.ghost:
                plan = self.client.preview('home')
                self.operation_id = self.client.execute(plan)['operation_id']
            return None
        if not value.clutch_active:
            self._hold()
            self.released = True
            if value.gripper is not None and not self.ghost:
                self.client.command('gripper', gripper_width=value.gripper * self.client.parameters['effective_robot_config']['gripper_max_width'],
                                    source_sequence=value.sequence, ttl_ms=100)
            return None
        if not self.released:
            return None
        p = snapshot['pose']
        robot = RobotState(timestamp_ns=snapshot['feedback_monotonic_ns'],
            joint_position=tuple(snapshot['q']), joint_velocity=tuple(snapshot['dq']), joint_torque=None,
            tcp_pose=Pose(Vector3(*p[:3]), Quaternion(*p[3:])), tcp_twist=None,
            gripper=snapshot['gripper_width'], external_wrench=None)
        if not self.active or self.recenter:
            self.policy.reset(value, robot)
            self.active = True
            self.recenter = False
        target = self.policy.update(value, robot, 0.)
        pose = [*target.target_pose.position.as_tuple(), *target.target_pose.orientation.as_tuple()]
        width = snapshot['gripper_width'] if target.gripper is None else target.gripper * self.client.parameters['effective_robot_config']['gripper_max_width']
        self.last_target = {'pose': pose, 'gripper_width': width, 'source_sequence': value.sequence}
        if not self.ghost:
            self.client.command('cartesian', **self.last_target, ttl_ms=100)
        return self.last_target

    def stop(self):
        self.source.stop()
        if not self.ghost:
            self.client.close()

    def run(self, duration=0):
        self.start()
        print(f'Phone UDP: {self.transport.local_address}; release clutch before activating; ghost={self.ghost}', flush=True)
        deadline = time.monotonic() + duration if duration else float('inf')
        try:
            while time.monotonic() < deadline:
                result = self.latest()
                if self.ghost and result:
                    print(json.dumps(result), flush=True)
        finally:
            self.stop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--bind-host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=5005)
    parser.add_argument('--mapping', choices=('phone_tool', 'phone_base'), default='phone_tool')
    parser.add_argument('--ghost', action='store_true')
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args(argv)
    from component_api.registry import Registry
    controller = Registry().create('source', 'phone.arkit', directory=args.run,
        bind_host=args.bind_host, port=args.port, mapping=args.mapping, ghost=args.ghost)
    try:
        controller.run(args.duration)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
