from __future__ import annotations

import pytest

from embodied.phone.controllers.cartesian_ruckig import CartesianServoConfig


def test_singularity_thresholds_are_explicit_pair() -> None:
    with pytest.raises(ValueError, match="set together"):
        CartesianServoConfig(singularity_warn=0.03)
    with pytest.raises(ValueError, match="set together"):
        CartesianServoConfig(singularity_stop=0.01)
    with pytest.raises(ValueError, match="exceed"):
        CartesianServoConfig(singularity_warn=0.01, singularity_stop=0.01)

    config = CartesianServoConfig(singularity_warn=0.03, singularity_stop=0.01)

    assert config.singularity_min_speed_scale == pytest.approx(0.25)
