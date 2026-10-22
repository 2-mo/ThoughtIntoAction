"""Repository locations shared by both Python runtimes; no SDK or config imports."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
CONTROL_CONFIG = PROJECT_ROOT / "configs/control"
ROBOT_CONFIG = PROJECT_ROOT / "configs/robots/arx5/robot.json"
CONTROL_RUNS = PROJECT_ROOT / "outputs/runtime/control"
CONTROL_ENVIRONMENT = PROJECT_ROOT / "configs/environments/control"
CONTROL_PYTHON = PROJECT_ROOT / "outputs/envs/control/bin/python"
CONTROL_BUILD = PROJECT_ROOT / "outputs/build/control"
