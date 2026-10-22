"""Reuse the iOS wire codec, session checks and relative mapping with the arx gateway."""
import argparse
from dataclasses import replace
import json
import hashlib
from pathlib import Path
import time

from embodied.devices.phone.inputs.arkit import ArkitInputSource
from embodied.devices.phone.core.mapping import PoseMapping, AxisMapping, MappingOutputFrame, MappingScales
from embodied.devices.phone.core.models import Pose, Quaternion, Vector3, RobotState, TrackingQuality
from embodied.devices.phone.policies.relative_pose import RelativePosePolicy
from embodied.devices.phone.transports.udp import UdpTransport, UdpTransportConfig
from embodied.contracts.runtime.control import ControlClient, ControlSession
from embodied.core.project import PROJECT_ROOT, port


class PhoneInput(ArkitInputSource):
    """Latch discrete controls before the upstream batch chooses the newest pose."""
    def __init__(self, transport, on_controls, **options):
        super().__init__(transport, **options)
        self.on_controls = on_controls
        self.on_message = None

    def _handle_payload(self, payload, received_at_ns):
        if payload.startswith(b'{'):
            if self.on_message is not None:
                self.on_message(payload)
            return None
        value = super()._handle_payload(payload, received_at_ns)
        if value is not None:
            self.on_controls(value)
        return value


def phone_settings():
    return json.loads((PROJECT_ROOT / 'configs/inputs/phone.json').read_text())


def phone_tool():
    settings = phone_settings()
    return PoseMapping(AxisMapping((1, 0, 2), (1, -1, 1)), settings['position_scale'],
                       settings['rotation_scale'], MappingOutputFrame.TOOL)


def phone_base():
    return replace(phone_tool(), output_frame=MappingOutputFrame.BASE)


class PhoneController:
    def __init__(self, directory=None, bind_host='127.0.0.1', port=5005, mapping='phone_tool', ghost=False):
        from embodied.contracts.registry import Registry
        registry = Registry()
        self.mapping = registry.create('mapping', mapping)
        settings = phone_settings()
        self.policy = RelativePosePolicy(self.mapping, mapping_provider=lambda: self.mapping,
            max_position_lead_m=settings['max_position_lead_m'],
            max_rotation_lead_rad=settings['max_rotation_lead_rad'])
        self.ghost = ghost
        self.mapping_name = mapping
        source = PROJECT_ROOT / 'src'
        files = [Path(__file__).resolve(), *sorted((source / 'embodied/devices/phone').rglob('*.py'))]
        # One digest over sorted (path, file hash) lines: a per-file map exceeds the gateway's 8 KiB metadata bound.
        tree = hashlib.sha256(''.join(f'{path.relative_to(source)} {hashlib.sha256(path.read_bytes()).hexdigest()}\n'
                                      for path in files).encode()).hexdigest()
        self.metadata = {'plugin': 'phone.arkit', 'protocol': 1, 'mapping': mapping,
                      **settings, 'pose_frame': 'base_tcp', 'quaternion': 'xyzw', 'gripper': 'metres',
                      'source_tree_sha256': tree, 'source_files': len(files)}
        if directory is None and not ghost:
            raise ValueError('A runtime is required for phone control')
        self.client = (None if directory is None else ControlClient(directory) if ghost else
                       ControlSession(directory, source='phone', purpose='teleop', metadata=self.metadata))
        self.transport = UdpTransport(UdpTransportConfig(bind_host=bind_host, bind_port=port, receive_timeout_s=.02))
        self.reset_request = lambda _: False
        self.source = registry.create('input', 'arkit', transport=self.transport, on_controls=self._controls,
            settings_callback=self._settings, estop_reset_request_callback=lambda packet: self.reset_request(packet))
        self.released = self.active = False
        self.session_id = None
        self.last_received = 0
        self.estop = self.home = False
        self.stop_input_time_ns = 0
        self.home_pressed = False
        self.recenter = self.recenter_pressed = False
        self.zero_requested = False
        self.operation_id = None
        self.last_target = None
        self.last_robot = None
        self.last_input = None
        self.input_count = 0

    def enable(self, directory):
        """Explicitly acquire an existing kernel without restarting the receiver."""
        if not self.ghost:
            raise ValueError('手机已持有控制权')
        if self.estop:
            raise ValueError('手机停止已锁存；松开 Clutch 后在手机点恢复')
        client = ControlSession(directory, source='phone', purpose='teleop', metadata=self.metadata)
        self.last_robot = client.status()
        client.__enter__()
        self.client, self.ghost = client, False
        self.released = self.active = self.home = self.recenter = False
        self.operation_id = None

    def disable(self):
        """Release control; keep the independent phone input available to monitors."""
        try:
            if not self.ghost and self.client is not None:
                self.client.close()
        finally:
            self.client, self.ghost = None, True
            self.active = self.released = self.home = self.recenter = False
            self.operation_id = None

    def _settings(self, scales):
        # Preserve the upstream policy's latch-at-reference semantics.
        accepted = MappingScales(min(scales.position_scale, 3.), scales.rotation_scale)
        self.mapping = self.mapping.with_scales(accepted)
        self.metadata.update(position_scale=accepted.position_scale, rotation_scale=accepted.rotation_scale)
        if not self.ghost:
            self.client.command('source-metadata', metadata={'mapping': self.mapping_name,
                'position_scale': accepted.position_scale, 'rotation_scale': accepted.rotation_scale,
                'activation': 'next clutch/reference'})
        return accepted

    def _controls(self, value):
        self.last_input = value
        self.last_received = time.monotonic()
        if value.estop_requested:
            self.stop_input_time_ns = value.timestamp_ns
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
        if value is not None:
            self.last_input = value
            self.input_count += 1
        if self.estop:
            if not self.ghost:
                self.disable()
                raise RuntimeError('手机停止已锁存，机械臂保持；松开 Clutch 后点恢复，再开始遥操')
            return None
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
        if self.client is None:
            return None
        snapshot = self.client.status()
        self.last_robot = snapshot
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
    parser.add_argument('--port', type=int, default=port('phone_udp'))
    parser.add_argument('--mapping', choices=('phone_tool', 'phone_base'), default='phone_tool')
    parser.add_argument('--ghost', action='store_true')
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args(argv)
    from embodied.contracts.registry import Registry
    controller = Registry().create('source', 'phone.arkit', directory=args.run,
        bind_host=args.bind_host, port=args.port, mapping=args.mapping, ghost=args.ghost)
    try:
        controller.run(args.duration)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
