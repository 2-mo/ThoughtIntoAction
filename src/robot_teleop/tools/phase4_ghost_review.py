"""Generate a static, hardware-free Phase 4 ghost mapping review."""

# ruff: noqa: E501

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from html import escape
from pathlib import Path

from robot_teleop.config import TeleopProfile, load_profile
from robot_teleop.controllers.cartesian_position import CartesianPositionController
from robot_teleop.core.capabilities import InputCapabilities
from robot_teleop.core.frames import (
    IDENTITY_QUATERNION,
    quaternion_angular_distance,
    quaternion_from_axis_angle,
)
from robot_teleop.core.models import (
    Pose,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from robot_teleop.core.pipeline import TeleopPipeline
from robot_teleop.core.safety import SafetyViolation
from robot_teleop.policies.no_compliance import NoCompliancePolicy
from robot_teleop.policies.relative_pose import RelativePosePolicy
from robot_teleop.robots.mock import MockRobotBackend, MockRobotConfig, RobotSafetyError
from robot_teleop.tools._common import validate_positive


@dataclass(frozen=True, slots=True)
class AxisReview:
    robot_axis: str
    source: str
    sign: int


@dataclass(frozen=True, slots=True)
class TranslationReview:
    input_axis: str
    input_delta_m: tuple[float, float, float]
    mapped_translation_m: tuple[float, float, float]
    dominant_robot_axis: str
    color: str


@dataclass(frozen=True, slots=True)
class RotationReview:
    input_axis: str
    input_angle_deg: float
    mapped_axis_vector: tuple[float, float, float]
    dominant_robot_axis: str
    mapped_angle_deg: float
    color: str


@dataclass(frozen=True, slots=True)
class TrajectorySample:
    label: str
    sequence: int
    clutch_active: bool
    recenter_requested: bool
    input_position_m: tuple[float, float, float]
    target_mode: str
    target_position_m: tuple[float, float, float] | None
    tcp_position_m: tuple[float, float, float]
    teleop_state: str


@dataclass(frozen=True, slots=True)
class SafetyRejectReview:
    label: str
    reason: str
    input_position_m: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class GhostReview:
    profile: str
    hardware_enabled: bool
    robot: str
    policy: str
    controller: str
    position_scale: float
    rotation_scale: float
    step_m: float
    rotation_deg: float
    axis_mapping: tuple[AxisReview, ...]
    workspace_minimum_m: tuple[float, float, float] | None
    workspace_maximum_m: tuple[float, float, float] | None
    translations: tuple[TranslationReview, ...]
    rotations: tuple[RotationReview, ...]
    trajectory: tuple[TrajectorySample, ...]
    safety_rejects: tuple[SafetyRejectReview, ...]
    operator_checklist: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


_INPUT_AXES: tuple[tuple[str, Vector3], ...] = (
    ("input_x", Vector3(1.0, 0.0, 0.0)),
    ("input_y", Vector3(0.0, 1.0, 0.0)),
    ("input_z", Vector3(0.0, 0.0, 1.0)),
)
_ROBOT_AXIS_NAMES = ("robot_x", "robot_y", "robot_z")
_COLORS = {
    "input_x": "#d62728",
    "input_y": "#2ca02c",
    "input_z": "#1f77b4",
}


def build_ghost_review(
    *,
    profile_path: Path,
    step_m: float = 0.1,
    rotation_deg: float = 30.0,
) -> GhostReview:
    """Build deterministic translation and orientation cases for review."""

    step = validate_positive("step_m", step_m)
    rotation_radians = math.radians(validate_positive("rotation_deg", rotation_deg))
    profile = load_profile(profile_path)
    workspace_minimum = profile.target_safety.workspace_minimum_m
    workspace_maximum = profile.target_safety.workspace_maximum_m
    return GhostReview(
        profile=str(profile_path),
        hardware_enabled=profile.hardware_requested,
        robot=profile.robot_name,
        policy=profile.policy_name,
        controller=profile.controller_name,
        position_scale=profile.mapping.position_scale,
        rotation_scale=profile.mapping.rotation_scale,
        step_m=step,
        rotation_deg=math.degrees(rotation_radians),
        axis_mapping=_axis_reviews(profile),
        workspace_minimum_m=None
        if workspace_minimum is None
        else _vector_values(workspace_minimum),
        workspace_maximum_m=None
        if workspace_maximum is None
        else _vector_values(workspace_maximum),
        translations=_translation_cases(profile, step),
        rotations=_rotation_cases(profile, rotation_radians),
        trajectory=_pipeline_trajectory(profile),
        safety_rejects=_safety_rejects(profile),
        operator_checklist=(
            "Move the phone along each input axis and compare the ghost direction.",
            "Rotate the phone about each input axis and compare the ghost rotation axis.",
            "Confirm Clutch press, release, re-press, and Recenter do not jump.",
            "Keep this review Mock-only; do not connect or enable robot hardware.",
        ),
    )


def write_ghost_review(
    *,
    profile_path: Path,
    output_dir: Path,
    step_m: float = 0.1,
    rotation_deg: float = 30.0,
) -> dict[str, object]:
    review = build_ghost_review(
        profile_path=profile_path,
        step_m=step_m,
        rotation_deg=rotation_deg,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "phase4_ghost_review.json"
    html_path = output_dir / "phase4_ghost_review.html"
    json_path.write_text(
        json.dumps(review.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    html_path.write_text(render_html(review), encoding="utf-8")
    return {
        "event": "phase4_ghost_review_written",
        "hardware_enabled": review.hardware_enabled,
        "profile": review.profile,
        "json": str(json_path),
        "html": str(html_path),
        "translations": len(review.translations),
        "rotations": len(review.rotations),
        "trajectory_samples": len(review.trajectory),
        "safety_rejects": len(review.safety_rejects),
    }


def render_html(review: GhostReview) -> str:
    mapping_rows = "\n".join(
        "<tr>"
        f"<td>{escape(axis.robot_axis)}</td>"
        f"<td>{escape(axis.source)}</td>"
        f"<td>{axis.sign}</td>"
        "</tr>"
        for axis in review.axis_mapping
    )
    translation_rows = "\n".join(
        "<tr>"
        f"<td>{_axis_dot(case.input_axis)}</td>"
        f"<td>{_numbers(case.mapped_translation_m)}</td>"
        f"<td>{escape(case.dominant_robot_axis)}</td>"
        "</tr>"
        for case in review.translations
    )
    rotation_rows = "\n".join(
        "<tr>"
        f"<td>{_axis_dot(case.input_axis)}</td>"
        f"<td>{escape(case.dominant_robot_axis)}</td>"
        f"<td>{case.mapped_angle_deg:.3f}</td>"
        "</tr>"
        for case in review.rotations
    )
    trajectory_rows = "\n".join(
        "<tr>"
        f"<td>{case.sequence}</td>"
        f"<td>{escape(case.label)}</td>"
        f"<td>{escape(case.target_mode)}</td>"
        f"<td>{escape(case.teleop_state)}</td>"
        f"<td>{_optional_numbers(case.target_position_m)}</td>"
        f"<td>{_numbers(case.tcp_position_m)}</td>"
        "</tr>"
        for case in review.trajectory
    )
    reject_rows = "\n".join(
        "<tr>"
        f"<td>{escape(case.label)}</td>"
        f"<td>{escape(case.reason)}</td>"
        f"<td>{_numbers(case.input_position_m)}</td>"
        "</tr>"
        for case in review.safety_rejects
    )
    checklist = "\n".join(
        f"<li>{escape(item)}</li>" for item in review.operator_checklist
    )
    workspace = (
        "not configured"
        if review.workspace_minimum_m is None or review.workspace_maximum_m is None
        else (
            "min "
            + _numbers(review.workspace_minimum_m)
            + ", max "
            + _numbers(review.workspace_maximum_m)
        )
    )
    raw_json = escape(json.dumps(review.as_dict(), indent=2, sort_keys=True))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Phase 4 Ghost Mapping Review</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: #172033;
      background: #ffffff;
      line-height: 1.45;
    }}
    main {{
      width: min(1180px, calc(100vw - 40px));
      margin: 28px auto 48px;
    }}
    h1 {{ margin: 0 0 6px; font-size: 28px; }}
    h2 {{ margin: 28px 0 12px; font-size: 18px; }}
    p {{ color: #657184; margin: 0 0 14px; }}
    .summary {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 10px;
      margin: 18px 0 24px;
    }}
    .summary div {{
      border: 1px solid #d8dee8;
      border-radius: 8px;
      padding: 10px 12px;
      background: #f7f9fc;
    }}
    .summary strong {{ display: block; font-size: 12px; color: #657184; }}
    .projections {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
      gap: 14px;
    }}
    svg {{
      width: 100%;
      height: auto;
      border: 1px solid #d8dee8;
      border-radius: 8px;
      background: #fff;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      margin: 8px 0 18px;
      font-size: 14px;
    }}
    th, td {{
      border-bottom: 1px solid #d8dee8;
      padding: 8px 10px;
      text-align: left;
      vertical-align: top;
    }}
    th {{ color: #657184; font-size: 12px; text-transform: uppercase; }}
    ul {{ margin: 8px 0 20px; padding-left: 20px; }}
    pre {{
      overflow: auto;
      border: 1px solid #d8dee8;
      border-radius: 8px;
      background: #0f1724;
      color: #eef4ff;
      padding: 14px;
      font-size: 12px;
    }}
    .dot {{
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 999px;
      margin-right: 7px;
      vertical-align: -1px;
    }}
  </style>
</head>
<body>
<main>
  <h1>Phase 4 Ghost Mapping Review</h1>
  <p>Hardware-free static review generated from the active profile. It is for
  Mock and Ghost direction confirmation only and cannot initialize robot hardware.</p>
  <section class="summary">
    <div><strong>Profile</strong>{escape(review.profile)}</div>
    <div><strong>Hardware Enabled</strong>{review.hardware_enabled}</div>
    <div><strong>Position Scale</strong>{review.position_scale:.3f}</div>
    <div><strong>Rotation Scale</strong>{review.rotation_scale:.3f}</div>
    <div><strong>Workspace</strong>{workspace}</div>
  </section>
  <h2>Ghost Direction Projections</h2>
  <div class="projections">
    {_projection_svg(review, "XY", 0, 1)}
    {_projection_svg(review, "XZ", 0, 2)}
    {_projection_svg(review, "YZ", 1, 2)}
  </div>
  <h2>Axis Mapping</h2>
  <table>
    <thead><tr><th>Robot axis</th><th>Input source</th><th>Sign</th></tr></thead>
    <tbody>{mapping_rows}</tbody>
  </table>
  <h2>Translation Checks</h2>
  <table>
    <thead><tr><th>Input move</th><th>Mapped robot delta, m</th><th>Dominant robot axis</th></tr></thead>
    <tbody>{translation_rows}</tbody>
  </table>
  <h2>Orientation Checks</h2>
  <table>
    <thead><tr><th>Input rotation</th><th>Mapped robot rotation axis</th><th>Mapped angle, deg</th></tr></thead>
    <tbody>{rotation_rows}</tbody>
  </table>
  <h2>Pipeline Trajectory</h2>
  <p>This trace is generated by the canonical pipeline and MockRobot backend.
  Target is the commanded Ghost pose; TCP is the resulting Mock state.</p>
  <div class="projections">
    {_trajectory_svg(review, "XY", 0, 1)}
    {_trajectory_svg(review, "XZ", 0, 2)}
    {_trajectory_svg(review, "YZ", 1, 2)}
  </div>
  <table>
    <thead><tr><th>Seq</th><th>Step</th><th>Target mode</th><th>Teleop state</th><th>Target pose, m</th><th>Mock TCP, m</th></tr></thead>
    <tbody>{trajectory_rows}</tbody>
  </table>
  <h2>Safety Rejects</h2>
  <table>
    <thead><tr><th>Case</th><th>Reason</th><th>Input pose, m</th></tr></thead>
    <tbody>{reject_rows or '<tr><td colspan="3">None</td></tr>'}</tbody>
  </table>
  <h2>Operator Checklist</h2>
  <ul>{checklist}</ul>
  <h2>Raw Review Data</h2>
  <pre>{raw_json}</pre>
</main>
</body>
</html>
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate a static Phase 4 Ghost mapping review.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/phone/profiles/arkit_mock.yaml"),
        help="Profile to visualize.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("build"),
        help="Directory for phase4_ghost_review.html and phase4_ghost_review.json.",
    )
    parser.add_argument("--step-m", type=float, default=0.1)
    parser.add_argument("--rotation-deg", type=float, default=30.0)
    args = parser.parse_args(argv)
    print(
        json.dumps(
            write_ghost_review(
                profile_path=args.config,
                output_dir=args.output_dir,
                step_m=args.step_m,
                rotation_deg=args.rotation_deg,
            ),
            sort_keys=True,
        )
    )
    return 0


def _axis_reviews(profile: TeleopProfile) -> tuple[AxisReview, ...]:
    return tuple(
        AxisReview(
            robot_axis=robot_axis,
            source=_source_name(profile.mapping.axes.robot_from_input[index]),
            sign=profile.mapping.axes.signs[index],
        )
        for index, robot_axis in enumerate(_ROBOT_AXIS_NAMES)
    )


def _pipeline_trajectory(
    profile: TeleopProfile,
) -> tuple[TrajectorySample, ...]:
    workspace_minimum = profile.target_safety.workspace_minimum_m
    workspace_maximum = profile.target_safety.workspace_maximum_m
    backend_config = MockRobotConfig(
        workspace_minimum_m=workspace_minimum or Vector3(0.0, -1.0, 0.0),
        workspace_maximum_m=workspace_maximum or Vector3(1.0, 1.0, 1.0),
    )
    backend = MockRobotBackend(backend_config)
    backend.connect()
    pipeline = TeleopPipeline(
        policy=RelativePosePolicy(profile.mapping),
        compliance=NoCompliancePolicy(),
        input_capabilities=InputCapabilities(
            pose_6dof=True,
            linear_velocity=False,
            angular_velocity=False,
            analog_gripper=True,
            haptics=False,
            dual_hand=False,
        ),
        safety_limits=profile.target_safety,
    )
    controller = CartesianPositionController(backend.capabilities)
    dt = 1.0 / 60.0
    timestamp_ns = 1_000_000_000
    input_path = (
        ("released baseline", Vector3(0.0, 0.0, 0.0), False, False),
        ("clutch press reference", Vector3(0.0, 0.0, 0.0), True, False),
        ("move input_z", Vector3(0.0, 0.0, 0.03), True, False),
        ("move input_z farther", Vector3(0.0, 0.0, 0.06), True, False),
        ("move input_x", Vector3(0.03, 0.0, 0.06), True, False),
        ("recenter at current pose", Vector3(0.03, 0.0, 0.06), True, True),
        ("post-recenter move", Vector3(0.05, 0.0, 0.06), True, False),
        ("release clutch", Vector3(0.05, 0.0, 0.06), False, False),
        ("reactivate reference", Vector3(0.05, 0.0, 0.06), True, False),
        ("move input_y", Vector3(0.05, 0.03, 0.06), True, False),
    )
    samples: list[TrajectorySample] = []
    try:
        for sequence, (label, position, clutch, recenter) in enumerate(input_path):
            input_state = TeleopInput(
                sequence=sequence,
                session_id=7,
                timestamp_ns=timestamp_ns,
                pose=Pose(position, IDENTITY_QUATERNION),
                linear_velocity=None,
                angular_velocity=None,
                clutch_active=clutch,
                recenter_requested=recenter,
                estop_requested=False,
                gripper=0.5,
                tracking_quality=TrackingQuality.NORMAL,
                source_name="phase4-ghost",
            )
            state = backend.state()
            target = pipeline.process(
                input_state,
                state,
                dt=dt,
                now_ns=timestamp_ns,
                input_dt=dt,
                is_new_input=True,
            )
            command = controller.update(target, state, dt)
            next_state = backend.send(command)
            samples.append(
                _trajectory_sample(
                    label=label,
                    sequence=sequence,
                    input_state=input_state,
                    target=target,
                    state=next_state,
                    teleop_state=pipeline.state.value,
                )
            )
            timestamp_ns += 16_666_667
    except RobotSafetyError as exc:
        raise RuntimeError(f"unexpected Mock backend safety error: {exc}") from exc
    finally:
        backend.disconnect()
    return tuple(samples)


def _safety_rejects(profile: TeleopProfile) -> tuple[SafetyRejectReview, ...]:
    workspace_minimum = profile.target_safety.workspace_minimum_m
    workspace_maximum = profile.target_safety.workspace_maximum_m
    if workspace_minimum is None or workspace_maximum is None:
        return ()
    backend = MockRobotBackend(
        MockRobotConfig(
            workspace_minimum_m=workspace_minimum,
            workspace_maximum_m=workspace_maximum,
        )
    )
    backend.connect()
    pipeline = TeleopPipeline(
        policy=RelativePosePolicy(profile.mapping),
        compliance=NoCompliancePolicy(),
        input_capabilities=InputCapabilities(
            pose_6dof=True,
            linear_velocity=False,
            angular_velocity=False,
            analog_gripper=True,
            haptics=False,
            dual_hand=False,
        ),
        safety_limits=profile.target_safety,
    )
    state = backend.state()
    reference = TeleopInput(
        sequence=0,
        session_id=11,
        timestamp_ns=1_000_000_000,
        pose=Pose(Vector3(0.0, 0.0, 0.0), IDENTITY_QUATERNION),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=True,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.5,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="phase4-ghost",
    )
    try:
        pipeline.process(
            reference,
            state,
            dt=1.0 / 60.0,
            now_ns=reference.timestamp_ns,
            input_dt=1.0 / 60.0,
            is_new_input=True,
        )
        outside = replace_input_pose(
            reference,
            sequence=1,
            timestamp_ns=1_000_000_001,
            position=Vector3(0.0, 0.0, 1.0),
        )
        try:
            pipeline.process(
                outside,
                backend.state(),
                dt=1.0 / 60.0,
                now_ns=outside.timestamp_ns,
                input_dt=1.0 / 60.0,
                is_new_input=True,
            )
        except SafetyViolation as exc:
            return (
                SafetyRejectReview(
                    label="workspace reject",
                    reason=str(exc),
                    input_position_m=_vector_values(outside.pose.position),
                ),
            )
        return ()
    finally:
        backend.disconnect()


def replace_input_pose(
    input_state: TeleopInput,
    *,
    sequence: int,
    timestamp_ns: int,
    position: Vector3,
) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=input_state.session_id,
        timestamp_ns=timestamp_ns,
        pose=Pose(position, input_state.pose.orientation),
        linear_velocity=input_state.linear_velocity,
        angular_velocity=input_state.angular_velocity,
        clutch_active=input_state.clutch_active,
        recenter_requested=input_state.recenter_requested,
        estop_requested=input_state.estop_requested,
        gripper=input_state.gripper,
        tracking_quality=input_state.tracking_quality,
        source_name=input_state.source_name,
    )


def _trajectory_sample(
    *,
    label: str,
    sequence: int,
    input_state: TeleopInput,
    target: RobotTarget,
    state: RobotState,
    teleop_state: str,
) -> TrajectorySample:
    target_position = (
        None
        if target.target_pose is None
        else _vector_values(target.target_pose.position)
    )
    return TrajectorySample(
        label=label,
        sequence=sequence,
        clutch_active=input_state.clutch_active,
        recenter_requested=input_state.recenter_requested,
        input_position_m=_vector_values(input_state.pose.position),
        target_mode=target.mode.value,
        target_position_m=target_position,
        tcp_position_m=_vector_values(state.tcp_pose.position),
        teleop_state=teleop_state,
    )


def _translation_cases(
    profile: TeleopProfile,
    step_m: float,
) -> tuple[TranslationReview, ...]:
    cases: list[TranslationReview] = []
    for input_axis, unit_vector in _INPUT_AXES:
        mapped = profile.mapping.map_delta(
            Pose(unit_vector * step_m, IDENTITY_QUATERNION),
        )
        cases.append(
            TranslationReview(
                input_axis=input_axis,
                input_delta_m=_vector_values(unit_vector * step_m),
                mapped_translation_m=_vector_values(mapped.position),
                dominant_robot_axis=_dominant_axis(mapped.position),
                color=_COLORS[input_axis],
            )
        )
    return tuple(cases)


def _rotation_cases(
    profile: TeleopProfile,
    rotation_radians: float,
) -> tuple[RotationReview, ...]:
    cases: list[RotationReview] = []
    for input_axis, unit_vector in _INPUT_AXES:
        mapped_axis = profile.mapping.axes.map_vector(unit_vector)
        mapped = profile.mapping.map_delta(
            Pose(
                Vector3(0.0, 0.0, 0.0),
                quaternion_from_axis_angle(unit_vector, rotation_radians),
            ),
        )
        cases.append(
            RotationReview(
                input_axis=input_axis,
                input_angle_deg=math.degrees(rotation_radians),
                mapped_axis_vector=_vector_values(mapped_axis),
                dominant_robot_axis=_dominant_axis(mapped_axis),
                mapped_angle_deg=math.degrees(
                    quaternion_angular_distance(
                        IDENTITY_QUATERNION,
                        mapped.orientation,
                    )
                ),
                color=_COLORS[input_axis],
            )
        )
    return tuple(cases)


def _projection_svg(
    review: GhostReview,
    label: str,
    first_index: int,
    second_index: int,
) -> str:
    size = 320.0
    center = size / 2.0
    margin = 44.0
    max_component = max(
        abs(value)
        for case in review.translations
        for value in case.mapped_translation_m
    )
    scale = (center - margin) / max(max_component, 1e-9)
    axis_names = ("X", "Y", "Z")
    arrows = "\n".join(
        _svg_arrow(case, label, first_index, second_index, center, scale)
        for case in review.translations
    )
    return f"""<svg viewBox="0 0 {size:.0f} {size:.0f}" role="img" aria-label="{label} ghost projection">
      <defs>
        <marker id="arrow-{label}" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
          <path d="M0,0 L8,4 L0,8 z" fill="#172033"></path>
        </marker>
      </defs>
      <text x="16" y="24" font-size="15" font-weight="700">{label} projection</text>
      <line x1="{margin}" y1="{center}" x2="{size - margin}" y2="{center}" stroke="#c5ccd8"></line>
      <line x1="{center}" y1="{size - margin}" x2="{center}" y2="{margin}" stroke="#c5ccd8"></line>
      <text x="{size - margin + 8}" y="{center + 5}" font-size="12">+{axis_names[first_index]}</text>
      <text x="{center + 6}" y="{margin - 10}" font-size="12">+{axis_names[second_index]}</text>
      <circle cx="{center}" cy="{center}" r="4" fill="#172033"></circle>
      {arrows}
    </svg>"""


def _trajectory_svg(
    review: GhostReview,
    label: str,
    first_index: int,
    second_index: int,
) -> str:
    size = 320.0
    margin = 44.0
    axis_names = ("X", "Y", "Z")
    positions: list[tuple[float, float, float]] = [
        sample.tcp_position_m for sample in review.trajectory
    ]
    positions.extend(
        sample.target_position_m
        for sample in review.trajectory
        if sample.target_position_m is not None
    )
    if not positions:
        return ""
    if (
        review.workspace_minimum_m is not None
        and review.workspace_maximum_m is not None
    ):
        minimum = review.workspace_minimum_m
        maximum = review.workspace_maximum_m
    else:
        minimum = (
            min(position[0] for position in positions),
            min(position[1] for position in positions),
            min(position[2] for position in positions),
        )
        maximum = (
            max(position[0] for position in positions),
            max(position[1] for position in positions),
            max(position[2] for position in positions),
        )
    first_span = max(maximum[first_index] - minimum[first_index], 1e-6)
    second_span = max(maximum[second_index] - minimum[second_index], 1e-6)
    width = size - 2.0 * margin
    height = size - 2.0 * margin

    def project(position: tuple[float, float, float]) -> tuple[float, float]:
        x = margin + (position[first_index] - minimum[first_index]) / first_span * width
        y = (
            size
            - margin
            - ((position[second_index] - minimum[second_index]) / second_span * height)
        )
        return x, y

    tcp_points = " ".join(
        f"{project(sample.tcp_position_m)[0]:.2f},{project(sample.tcp_position_m)[1]:.2f}"
        for sample in review.trajectory
    )
    target_points = " ".join(
        f"{project(sample.target_position_m)[0]:.2f},{project(sample.target_position_m)[1]:.2f}"
        for sample in review.trajectory
        if sample.target_position_m is not None
    )
    workspace_rect = ""
    if (
        review.workspace_minimum_m is not None
        and review.workspace_maximum_m is not None
    ):
        workspace_minimum = review.workspace_minimum_m
        workspace_maximum = review.workspace_maximum_m
        top_left = project(
            (
                workspace_maximum[0] if second_index == 0 else workspace_minimum[0],
                workspace_maximum[1] if second_index == 1 else workspace_minimum[1],
                workspace_maximum[2] if second_index == 2 else workspace_minimum[2],
            )
        )
        bottom_right = project(
            (
                workspace_minimum[0] if second_index == 0 else workspace_maximum[0],
                workspace_minimum[1] if second_index == 1 else workspace_maximum[1],
                workspace_minimum[2] if second_index == 2 else workspace_maximum[2],
            )
        )
        workspace_rect = (
            f'<rect x="{top_left[0]:.2f}" y="{top_left[1]:.2f}" '
            f'width="{bottom_right[0] - top_left[0]:.2f}" '
            f'height="{bottom_right[1] - top_left[1]:.2f}" '
            'fill="#edf4ff" stroke="#8aa9d6"></rect>'
        )
    return f"""<svg viewBox="0 0 {size:.0f} {size:.0f}" role="img" aria-label="{label} pipeline trajectory">
      <text x="16" y="24" font-size="15" font-weight="700">{label} trajectory</text>
      {workspace_rect}
      <polyline points="{tcp_points}" fill="none" stroke="#172033" stroke-width="2"></polyline>
      <polyline points="{target_points}" fill="none" stroke="#e07a1f" stroke-width="2" stroke-dasharray="5 3"></polyline>
      <line x1="{margin}" y1="{size - margin + 10}" x2="{margin + 22}" y2="{size - margin + 10}" stroke="#172033" stroke-width="2"></line>
      <text x="{margin + 28}" y="{size - margin + 14}" font-size="11">Mock TCP</text>
      <line x1="{margin + 92}" y1="{size - margin + 10}" x2="{margin + 114}" y2="{size - margin + 10}" stroke="#e07a1f" stroke-width="2" stroke-dasharray="5 3"></line>
      <text x="{margin + 120}" y="{size - margin + 14}" font-size="11">Target</text>
      <text x="{size - margin + 4}" y="{size - margin + 4}" font-size="11">+{axis_names[first_index]}</text>
      <text x="{margin - 2}" y="{margin - 8}" font-size="11">+{axis_names[second_index]}</text>
    </svg>"""


def _svg_arrow(
    case: TranslationReview,
    label: str,
    first_index: int,
    second_index: int,
    center: float,
    scale: float,
) -> str:
    x = center + case.mapped_translation_m[first_index] * scale
    y = center - case.mapped_translation_m[second_index] * scale
    text_label = case.input_axis.replace("input_", "+")
    return (
        f'<line x1="{center:.3f}" y1="{center:.3f}" x2="{x:.3f}" y2="{y:.3f}" '
        f'stroke="{case.color}" stroke-width="3" marker-end="url(#arrow-{label})"></line>'
        f'<text x="{x + 6:.3f}" y="{y - 6:.3f}" font-size="12" fill="{case.color}">'
        f"{escape(text_label)}</text>"
    )


def _source_name(index: int) -> str:
    return _INPUT_AXES[index][0]


def _vector_values(value: Vector3) -> tuple[float, float, float]:
    return (value.x, value.y, value.z)


def _dominant_axis(value: Vector3) -> str:
    values = value.as_tuple()
    index = max(range(3), key=lambda item: abs(values[item]))
    if abs(values[index]) <= 1e-12:
        return "zero"
    prefix = "+" if values[index] > 0.0 else "-"
    return f"{prefix}{('X', 'Y', 'Z')[index]}"


def _numbers(values: tuple[float, float, float]) -> str:
    return "[" + ", ".join(f"{value:.4f}" for value in values) + "]"


def _optional_numbers(values: tuple[float, float, float] | None) -> str:
    return "none" if values is None else _numbers(values)


def _axis_dot(input_axis: str) -> str:
    return (
        f'<span class="dot" style="background:{escape(_COLORS[input_axis])}"></span>'
        f"{escape(input_axis)}"
    )
