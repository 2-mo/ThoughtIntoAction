"""Canonical repository paths shared by runtime modules."""

from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SOURCE_ROOT.parent
ASSETS_ROOT = PROJECT_ROOT / "assets" / "arx_motion"
CALIBRATIONS_ROOT = PROJECT_ROOT / "calibrations" / "arx_motion"
CONFIG_ROOT = PROJECT_ROOT / "configs" / "arx_motion"
WORK_ROOT = Path.home() / "work"
DATASETS_ROOT = WORK_ROOT / "data" / "arx5"
PRE_WEIGHTS_ROOT = WORK_ROOT / "pre_weights"
OUTPUTS_ROOT = PROJECT_ROOT / "outputs" / "arx_motion"
RUNTIME_ROOT = PROJECT_ROOT / "runtime" / "arx_motion"
WEIGHTS_ROOT = ASSETS_ROOT / "weights"
FRONTEND_ROOT = PROJECT_ROOT / "apps" / "console"
HANDEYE_ROOT = SOURCE_ROOT / "hand_eye_calibration"
