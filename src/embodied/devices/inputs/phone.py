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
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.control.homing import ReleaseThenHome


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


def phone_calibrated():
    return replace(phone_tool(), output_frame=MappingOutputFrame.CALIBRATED)


class PhoneController:
    def __init__(self, directory=None, bind_host='127.0.0.1', port=5005, mapping='phone_calibrated', ghost=False):
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
        self.preset = None
        self.homing = None
        self.calibrated_runtime_id = None
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
        if self.calibrated_runtime_id != client.identity['runtime_id']:
            self.clear_calibration()
        self.released = self.active = self.home = self.recenter = False
        self.operation_id = None
        self.preset = None
        self.homing = None

    def disable(self):
        """Release control; keep the independent phone input available to monitors."""
        try:
            if not self.ghost and self.client is not None:
                self.client.close()
        finally:
            self.client, self.ghost = None, True
            self.active = self.released = self.home = self.recenter = False
            self.operation_id = None
            self.homing = None
            if self.preset_busy:
                self.preset.update(phase='cancelled', message='动作已停止，机械臂保持')

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
        if (self.active or self.preset_busy) and not self.ghost:
            self.client.command('hold')
        if self.preset_busy:
            self.preset.update(phase='cancelled', message='输入中断，动作已停止；请重新选择')
            self.operation_id = None
            self.homing = None
        self.active = False

    @property
    def preset_busy(self):
        return self.preset is not None and self.preset['phase'] in ('opening', 'moving')

    def request_preset(self, name):
        """Use the current phone lease; opening and arm motion are separate steps."""
        if name not in ('home', 'pregrasp'):
            raise ValueError('未知预设动作')
        if self.ghost or self.client is None:
            raise ValueError('先开始手机遥操，再选择预设动作')
        value = self.last_input
        if (self.estop or value is None or time.monotonic() - self.last_received > .1
                or value.tracking_quality != TrackingQuality.NORMAL or value.clutch_active):
            raise ValueError('松开 Clutch，等待手机正常追踪后再选择预设动作')
        if self.preset_busy:
            raise ValueError('动作正在执行；停止后可重新选择，不排队重复动作')
        if name == 'pregrasp' and self.client.identity.get('plugins', {}).get('robot') not in ('arx.real', 'arx.sim'):
            raise ValueError('当前机器人未配置预抓取位姿')
        self.last_robot = self.client.status()
        self._hold()
        self.released = self.home = False
        self.operation_id = None
        config = self.client.parameters['effective_robot_config']
        opening = name == 'home' and config['gripper_max_width'] > 0
        self.preset = {'name': name, 'phase': 'opening' if opening else 'moving',
                       'message': '先张开夹爪，确认松开后回零' if opening else '正在回零' if name == 'home' else '正在前往预抓取位'}
        self.homing = ReleaseThenHome(self.client) if name == 'home' else None

    def clear_calibration(self):
        self.policy.clear_alignment()
        self.calibrated_runtime_id = None
        self.metadata.pop('input_to_base_quaternion_xyzw', None)

    def request_alignment(self, robot_state, runtime_id):
        value = self.last_input
        if (self.estop or self.preset_busy or value is None or value.clutch_active
                or time.monotonic() - self.last_received > .1 or value.tracking_quality != TrackingQuality.NORMAL):
            raise ValueError('松开 Clutch 并等待动作结束和正常追踪后校准')
        if self.mapping.output_frame is not MappingOutputFrame.CALIBRATED:
            raise ValueError('当前映射不是手动校准模式')
        if time.monotonic_ns() - robot_state['feedback_monotonic_ns'] > 500_000_000:
            raise ValueError('机械臂反馈过期，请恢复连接后对齐')
        self._hold()
        basis = self.policy.calibrate(value.pose.orientation, Quaternion(*robot_state['pose'][3:]))
        self.calibrated_runtime_id = runtime_id
        self.metadata['input_to_base_quaternion_xyzw'] = list(basis.as_tuple())
        if not self.ghost:
            self.client.command('source-metadata', metadata={
                'input_to_base_quaternion_xyzw': list(basis.as_tuple()),
                'calibration_method': 'manual_current_phone_and_tcp'})
        self.released = False
        self.preset = None

    def _advance_preset(self, snapshot):
        name = self.preset['name']
        label = '回零' if name == 'home' else '预抓取'
        if self.homing is not None:
            op = self.homing.step(snapshot)
            self.operation_id = self.homing.operation_id
            if self.homing.phase == 'moving':
                self.preset.update(phase='moving', message='夹爪已张开，正在回零')
            if op is None:
                return
        else:
            if self.operation_id is None:
                plan = self.client.preview('movej', q=list(DEFAULT_GRASP_READY_JOINTS))
                self.operation_id = self.client.execute(plan)['operation_id']
                return
            op = next((op for op in snapshot['operations'] if op['operation_id'] == self.operation_id), None)
            if op is None:
                raise RuntimeError(f'{label}状态丢失，未确认到位')
        if op['status'] != 'running':
            if op['status'] != 'completed':
                raise RuntimeError(f'{label}未完成：{op.get("reason") or op["status"]}')
            self.operation_id = None
            self.preset.update(phase='completed', message=f'{label}已完成；按住 Clutch 可继续遥操')
            self.released = self.active = False

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
            if now - self.last_received > .5:
                self.clear_calibration()
            return None
        self.last_received = now
        if value.session_id != self.session_id:
            self._hold()
            self.clear_calibration()
            self.session_id, self.released = value.session_id, False
        if value.tracking_quality != TrackingQuality.NORMAL:
            self._hold()
            if value.tracking_quality == TrackingQuality.UNAVAILABLE:
                self.clear_calibration()
            self.released = False
            return None
        if self.client is None:
            self.home = False
            return None
        snapshot = self.client.status()
        self.last_robot = snapshot
        if self.preset_busy:
            self.home = False
            self._advance_preset(snapshot)
            return None
        if self.home:
            self.home = False
            if not self.ghost:
                self.request_preset('home')
            return None
        if not value.clutch_active:
            self._hold()
            self.released = True
            return None
        if not self.released:
            return None
        if self.mapping.output_frame is MappingOutputFrame.CALIBRATED and self.policy.rotation_alignment is None:
            self._hold()
            return None
        p = snapshot['pose']
        robot = RobotState(timestamp_ns=snapshot['feedback_monotonic_ns'],
            joint_position=tuple(snapshot['q']), joint_velocity=tuple(snapshot['dq']), joint_torque=None,
            tcp_pose=Pose(Vector3(*p[:3]), Quaternion(*p[3:])), tcp_twist=None,
            gripper=snapshot['gripper_width'], external_wrench=None)
        if not self.active or self.recenter:
            self.policy.reset(value, robot)
            self.active = True
            self.preset = None
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
    parser.add_argument('--mapping', choices=('phone_calibrated', 'phone_tool', 'phone_base'), default='phone_calibrated')
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
