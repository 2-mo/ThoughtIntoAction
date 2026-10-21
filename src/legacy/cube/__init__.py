"""D455-guided cube pick, rotate, and place experiment."""

# Keep package import lightweight. Perception is explicitly imported by the
# experiment entry point because the control-only .venv intentionally has no
# OpenCV or pyrealsense2.
from legacy.cube.config import ExperimentConfig, load_config

__all__ = ["ExperimentConfig", "load_config"]
