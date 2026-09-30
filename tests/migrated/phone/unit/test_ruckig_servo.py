from __future__ import annotations

import pytest

from robot_teleop.controllers.ruckig_servo import (
    JointServoLimits,
    JointServoState,
    RuckigJointServo,
)

ruckig = pytest.importorskip("ruckig")


def limits() -> JointServoLimits:
    return JointServoLimits(
        minimum_position=(-2.0,),
        maximum_position=(2.0,),
        maximum_velocity=(2.0,),
        maximum_acceleration=(4.0,),
        maximum_jerk=(20.0,),
    )


def test_latest_target_replaces_previous_target_and_preserves_bounds() -> None:
    servo = RuckigJointServo(
        JointServoState((0.0,), (0.0,), (0.0,)),
        limits(),
        period_s=0.005,
        ruckig_module=ruckig,
    )
    servo.set_target((0.25,))
    servo.step()
    servo.set_target((0.9,))

    samples = [servo.step() for _ in range(300)]

    assert samples[-1].position[0] == pytest.approx(0.9, abs=2e-3)
    assert all(-2.0 <= sample.position[0] <= 2.0 for sample in samples)
    assert all(abs(sample.velocity[0]) <= 2.0 + 1e-7 for sample in samples)
    assert all(abs(sample.acceleration[0]) <= 4.0 + 1e-6 for sample in samples)
    assert all(abs(sample.jerk[0]) <= 20.0 + 1e-5 for sample in samples)


def test_brake_switches_to_zero_velocity_without_a_history_queue() -> None:
    servo = RuckigJointServo(
        JointServoState((0.0,), (0.0,), (0.0,)),
        limits(),
        period_s=0.005,
        ruckig_module=ruckig,
    )
    servo.set_target((1.0,))
    for _ in range(30):
        servo.step()
    servo.brake()

    samples = [servo.step() for _ in range(300)]

    assert servo.braking is True
    assert abs(samples[-1].velocity[0]) < 1e-3
    assert samples[-1].position[0] < 1.0


def test_invalid_target_and_limits_are_rejected() -> None:
    with pytest.raises(ValueError, match="below"):
        JointServoLimits(
            minimum_position=(1.0,),
            maximum_position=(1.0,),
            maximum_velocity=(1.0,),
            maximum_acceleration=(1.0,),
            maximum_jerk=(1.0,),
        )

    servo = RuckigJointServo(
        JointServoState((0.0,), (0.0,), (0.0,)),
        limits(),
        period_s=0.005,
        ruckig_module=ruckig,
    )
    with pytest.raises(ValueError, match="outside"):
        servo.set_target((3.0,))
    with pytest.raises(ValueError, match="velocity"):
        servo.set_velocity_target((3.0,))
    with pytest.raises(ValueError, match="acceleration"):
        RuckigJointServo(
            JointServoState((0.0,), (0.0,), (5.0,)),
            limits(),
            period_s=0.005,
            ruckig_module=ruckig,
        )
