"""Canonical session, policy, filtering, compliance, and safety pipeline."""

from __future__ import annotations

from dataclasses import replace

from embodied.devices.phone.core.capabilities import InputCapabilities
from embodied.devices.phone.core.filtering import IdentityTargetFilter
from embodied.devices.phone.core.models import (
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    hold_target,
    joint_zero_target,
    stop_target,
)
from embodied.devices.phone.core.ports import CompliancePolicy, ControlPolicy, TargetFilter
from embodied.devices.phone.core.safety import TargetSafetyGate, TargetSafetyLimits
from embodied.devices.phone.core.session import SessionAction, TeleopSession
from embodied.devices.phone.core.state_machine import TeleopEvent, TeleopState, TeleopStateMachine


class TeleopPipeline:
    def __init__(
        self,
        *,
        policy: ControlPolicy,
        compliance: CompliancePolicy,
        input_capabilities: InputCapabilities,
        safety_limits: TargetSafetyLimits,
        target_filter: TargetFilter | None = None,
    ) -> None:
        if not input_capabilities.pose_6dof:
            raise ValueError("Relative teleoperation requires a 6DoF pose capability")
        self._policy = policy
        self._compliance = compliance
        self._input_capabilities = input_capabilities
        self._filter = target_filter or IdentityTargetFilter()
        self._safety = TargetSafetyGate(safety_limits)
        self._session = TeleopSession()
        self._machine = TeleopStateMachine()
        self._joint_zero_active = False

    @property
    def state(self) -> TeleopState:
        return self._machine.state

    def process(
        self,
        input_state: TeleopInput,
        robot_state: RobotState,
        *,
        dt: float,
        now_ns: int,
        input_dt: float | None = None,
        is_new_input: bool = True,
    ) -> RobotTarget:
        if input_state.tracking_quality is not TrackingQuality.UNAVAILABLE:
            self._machine.transition(TeleopEvent.CONNECT)
        if input_state.estop_requested:
            self._machine.transition(TeleopEvent.ESTOP)
            return stop_target(now_ns)
        if self._machine.state in (TeleopState.ESTOP, TeleopState.FAULT):
            return stop_target(now_ns)

        if (
            input_state.home_requested or input_state.joint_zero_requested
        ) and is_new_input:
            self._joint_zero_active = True
            self._session.reset(require_release=True)
            self._safety.reset(robot_state.tcp_pose)
            self._filter.reset(robot_state)
            self._machine.transition(TeleopEvent.RELEASE)

        if self._joint_zero_active:
            if max(abs(value) for value in robot_state.joint_position) <= 0.002:
                self._joint_zero_active = False
                return hold_target(now_ns, gripper=input_state.gripper)
            self._machine.transition(TeleopEvent.RELEASE)
            return joint_zero_target(
                now_ns,
                len(robot_state.joint_position),
                gripper=input_state.gripper,
            )

        decision = self._session.observe(input_state)
        if decision.action is SessionAction.DEGRADED:
            self._machine.transition(TeleopEvent.TRACKING_LIMITED)
            return hold_target(now_ns, gripper=input_state.gripper)
        if decision.action is SessionAction.TRACKING_LOST:
            self._machine.transition(TeleopEvent.TRACKING_UNAVAILABLE)
            return hold_target(now_ns, gripper=input_state.gripper)
        self._machine.transition(TeleopEvent.TRACKING_NORMAL)

        if decision.action is SessionAction.SESSION_CHANGED:
            self._machine.transition(TeleopEvent.RELEASE)
            self._safety.reset(robot_state.tcp_pose)
            self._filter.reset(robot_state)
            return hold_target(now_ns, gripper=input_state.gripper)
        if decision.action in (SessionAction.IDLE, SessionAction.RELEASE):
            if (
                decision.action is SessionAction.RELEASE
                or not input_state.clutch_active
            ) and self._machine.state is TeleopState.ACTIVE:
                self._machine.transition(TeleopEvent.RELEASE)
            return hold_target(now_ns, gripper=input_state.gripper)

        if decision.action in (SessionAction.ACTIVATE, SessionAction.RECENTER):
            self._policy.reset(input_state, robot_state)
            self._safety.reset(robot_state.tcp_pose)
            self._filter.reset(robot_state)
        self._machine.transition(TeleopEvent.ACTIVATE)
        target = self._policy.update(input_state, robot_state, dt)
        target = replace(target, timestamp_ns=now_ns)
        if not self._input_capabilities.analog_gripper:
            target = replace(target, gripper=None)
        target = self._filter.apply(target, robot_state, dt)
        target = self._compliance.modify(target, robot_state, dt)
        return self._safety.validate(
            target,
            robot_state,
            dt if input_dt is None else input_dt,
            is_new_input=is_new_input,
        )

    def hold(self, now_ns: int) -> RobotTarget:
        if (
            not self._machine.latched
            and self._machine.state is not TeleopState.DISCONNECTED
        ):
            self._machine.transition(TeleopEvent.RELEASE)
        return hold_target(now_ns)

    def suspend(self) -> None:
        """Pause external control and require a fresh clutch edge to resume."""
        if (
            not self._machine.latched
            and self._machine.state is not TeleopState.DISCONNECTED
        ):
            self._machine.transition(TeleopEvent.RELEASE)
        self._session.reset(require_release=True)

    def fault(self) -> None:
        self._machine.transition(TeleopEvent.FAULT)

    def disconnect(self) -> None:
        self._machine.transition(TeleopEvent.DISCONNECT)
        self._session.reset(require_release=True)
        self._joint_zero_active = False

    def reset_latches(self) -> None:
        self._machine.transition(TeleopEvent.RESET)
        self._session.reset(require_release=True)
