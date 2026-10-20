import json
from pathlib import Path

import pytest

from embodied.phone.tools.phase4_ghost_review import (
    build_ghost_review,
    render_html,
    write_ghost_review,
)


def test_phase4_ghost_model_matches_arkit_mock_mapping() -> None:
    review = build_ghost_review(
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
        step_m=0.1,
        rotation_deg=30.0,
    )

    translations = {
        case.input_axis: case.mapped_translation_m for case in review.translations
    }
    rotations = {case.input_axis: case.dominant_robot_axis for case in review.rotations}

    assert review.hardware_enabled is False
    assert translations["input_x"] == pytest.approx((0.0, -0.2, 0.0))
    assert translations["input_y"] == pytest.approx((0.2, 0.0, 0.0))
    assert translations["input_z"] == pytest.approx((0.0, 0.0, 0.2))
    assert rotations == {
        "input_x": "-Y",
        "input_y": "+X",
        "input_z": "+Z",
    }
    assert len(review.trajectory) == 10
    assert review.trajectory[0].target_mode == "hold"
    assert review.trajectory[5].label == "recenter at current pose"
    assert review.trajectory[5].tcp_position_m == pytest.approx(
        review.trajectory[4].tcp_position_m
    )
    assert len(review.safety_rejects) == 1
    assert "outside the workspace" in review.safety_rejects[0].reason


def test_phase4_ghost_review_writes_static_artifacts(tmp_path: Path) -> None:
    report = write_ghost_review(
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
        output_dir=tmp_path,
        step_m=0.05,
        rotation_deg=20.0,
    )

    html_path = Path(str(report["html"]))
    json_path = Path(str(report["json"]))
    assert html_path.exists()
    assert json_path.exists()
    data = json.loads(json_path.read_text(encoding="utf-8"))
    html = html_path.read_text(encoding="utf-8")
    assert data["hardware_enabled"] is False
    assert "Phase 4 Ghost Mapping Review" in html
    assert "Mock and Ghost direction confirmation only" in html
    assert "Pipeline Trajectory" in html
    assert "Safety Rejects" in html


def test_phase4_ghost_html_includes_review_tables() -> None:
    review = build_ghost_review(
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
    )

    html = render_html(review)

    assert "Translation Checks" in html
    assert "Orientation Checks" in html
    assert "robot_x" in html
