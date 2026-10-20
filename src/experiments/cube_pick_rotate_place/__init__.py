"""D455-guided cube pick, rotate, and place experiment."""

# Keep package import lightweight. Perception is explicitly imported by the
# experiment entry point because the control-only .venv intentionally has no
# OpenCV or pyrealsense2.
from experiments.cube_pick_rotate_place.config import ExperimentConfig, load_config

__all__ = ["ExperimentConfig", "load_config"]
