"""Blocked-jaw regression and actual adapter packets, without constructing CAN."""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from embodied.contracts.control import ControlCommand, Mode, RobotState
from embodied.devices.robots.arx5.gripper import GripperPositionServo
from embodied.devices.robots.arx5.profile import RobotConfig
from embodied.devices.robots.arx5.real import RealBackend


def servo(sign=-1):
    return GripperPositionServo(open_readout=sign * 3.4, width=.082, kp=5., kd=.1,
                                effort_limit=.15, speed=.3, dt=.004)


@pytest.mark.parametrize('sign', [-1, 1])
def test_blocked_jaw_caps_total_effort_without_reaching_absolute_goal(sign):
    control = servo(sign)
    efforts = [control.update(0., .03, 0.) for _ in range(2500)]
    assert max(map(abs, efforts)) <= .15
    assert efforts[-1] == pytest.approx(-sign * .15)
    assert control.limited
    # No accumulated demand through the object; opening reverses promptly.
    for _ in range(4):
        effort = control.update(.082, .03, 0.)
    assert effort * sign > 0


def test_total_effort_includes_damping_and_reference_resets_to_feedback():
    control = servo()
    assert abs(control.update(.082, .03, -20.)) <= .15
    control.reset()
    assert control.update(.025, .025, 0.) == 0
    with pytest.raises(ValueError, match='finite feedback'):
        control.update(.02, .03, float('nan'))


def test_waypoint_keeps_configured_speed_and_no_force_at_reached_goal():
    control = servo()
    actual = .04
    for _ in range(10):
        control.update(.082, actual, 0.)
        assert control.reference - actual <= .3 * .004 + 1e-9
        actual = control.reference
    control.reset()
    assert control.update(actual, actual, 0.) == 0


@pytest.mark.parametrize('direction', [-1, 1])
def test_moving_at_reference_speed_is_not_braked_toward_zero(direction):
    control = GripperPositionServo(open_readout=-3.4, width=.082, kp=5., kd=.2,
                                  effort_limit=.15, speed=.3, dt=.004)
    control.reference = .04
    effort = control.update(.082 if direction > 0 else 0., .04, direction * .3)
    assert control.reference_velocity == pytest.approx(direction * .3)
    assert effort * direction < 0  # negative motor torque opens this jaw
    assert abs(effort) <= .15


def test_blocked_reference_stops_advancing_and_endpoint_restores_damping():
    control = servo()
    for _ in range(20):
        control.update(0, .03, 0)
    assert control.reference_velocity == 0
    control.reset()
    assert control.reference_velocity == 0
    effort = control.update(.04, .04, .03)
    assert effort > 0  # measured opening velocity is braked at a stationary goal
    assert abs(effort) <= .15


class Packet:
    def __init__(self, count):
        self.q, self.dq, self.tau = (np.zeros(count) for _ in range(3))
        self.gripper_kp = self.gripper_kd = 0.

    def pos(self): return self.q
    def vel(self): return self.dq
    def torque(self): return self.tau
    def kp(self): return self.q
    def kd(self): return self.dq


def adapter():
    backend = RealBackend.__new__(RealBackend)
    backend.config = RobotConfig()
    backend._sdk = SimpleNamespace(Gain=Packet, JointState=Packet)
    backend._automatic_gravity, backend._stopped = True, False
    backend._sdk_torque_ceiling, backend._gripper_effort_max = np.full(6, 100.), 1.5
    backend._gripper_position, backend._telemetry = servo(), {}
    state = RobotState(np.zeros(6), np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1]),
        .03, 0., gripper_velocity=0., gripper_effort=.15)
    backend.read_state = lambda now: state
    controller = SimpleNamespace(dq_target=np.zeros(6), ddq_target=np.zeros(6))
    def update(command, q, now):
        controller.gripper_mode = command.gripper_mode or command.mode
        controller.gripper_target = command.gripper_target
        return q
    controller.update = update
    backend.controller = controller
    hardware = SimpleNamespace()
    hardware.set_joint_cmd = lambda packet: setattr(hardware, 'packet', packet)
    hardware.set_gain = lambda gain: setattr(hardware, 'gain', gain)
    hardware.send_recv_once = lambda: None
    hardware.get_joint_cmd = lambda: hardware.packet
    hardware.get_gain = lambda: hardware.gain
    backend._require_open = lambda: hardware
    return backend, hardware


def test_adapter_removes_native_pd_so_it_cannot_bypass_effort_cap():
    backend, hw = adapter()
    command = ControlCommand(Mode.COMMAND_FOLLOWING, gripper_target=0., joint_target=np.zeros(6))
    for tick in range(100):
        backend.step(command, tick / 250)
        assert hw.gain.gripper_kp == hw.gain.gripper_kd == 0.
        assert 0 < hw.packet.gripper_torque <= .15
        np.testing.assert_array_equal(hw.gain.kp(), backend.config.kp)
        np.testing.assert_array_equal(hw.gain.kd(), backend.config.kd)
    backend.step(replace(command, gripper_mode=Mode.FREE_DRIVE), 1.)
    assert hw.packet.gripper_torque == 0
    assert backend._gripper_position.reference is None
    # Explicit grasp remains a separate force command with its declared limit.
    backend.step(replace(command, gripper_effort=.12), 1.1)
    assert hw.packet.gripper_torque == .12
    assert hw.gain.gripper_kp == hw.gain.gripper_kd == 0.


@pytest.mark.parametrize('speed', [.12, .3, .31])
def test_configured_gripper_speed_reaches_servo_and_rejects_widening_sdk_limit(monkeypatch, speed):
    robot = SimpleNamespace(joint_vel_max=np.full(6, 3.), joint_torque_max=np.full(6, 100.),
        gripper_vel_max=.3, gripper_torque_max=1.5, motor_id=list(range(6)), gripper_motor_id=6)
    control = SimpleNamespace(default_gripper_kp=5., default_gripper_kd=.2)
    def factory(value):
        return SimpleNamespace(get_instance=lambda: SimpleNamespace(get_config=lambda *args: value))
    def unexpected_hardware(*args):
        pytest.fail('offline configuration check must never construct hardware')
    sdk = SimpleNamespace(RobotConfigFactory=factory(robot), ControllerConfigFactory=factory(control),
                          quest_gripper_turns_version=1, Arx5JointController=unexpected_hardware)
    monkeypatch.setitem(sys.modules, 'arx5_interface', sdk)
    monkeypatch.setitem(sys.modules, 'embodied.devices.robots.arx5.kinematics', SimpleNamespace(
        X5Model=lambda config: SimpleNamespace(urdf_path=Path('offline.urdf'), tcp_link='tcp'),
        JointTargetController=lambda config, model: SimpleNamespace()))
    monkeypatch.setenv('QUEST_ARX_GRIPPER_TURNS', '1')
    backend = RealBackend.__new__(RealBackend)
    config = replace(RobotConfig(), gripper_velocity_max_m_s=speed)
    if speed > .3:
        with pytest.raises(ValueError, match='gripper_velocity_max_m_s'):
            backend.__init__(config, cancel_requested=lambda: True)
    else:
        with pytest.raises(InterruptedError, match='before SDK construction'):
            backend.__init__(config, cancel_requested=lambda: True)
        assert backend._gripper_position.speed == speed
        assert backend.runtime_parameters()['gripper_position_control']['velocity_max_m_s'] == speed
