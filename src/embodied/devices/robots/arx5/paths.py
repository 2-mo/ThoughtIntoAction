"""ARX X5 resources; resolving local models never loads an SDK or dependency manifest."""
from pathlib import Path
from embodied.contracts.paths import PROJECT_ROOT

X5_2025_URDF = PROJECT_ROOT / "assets/robots/x5/X5_2025_hybrid.urdf"


def official_sdk_python_root():
    """The optional official SDK's installed native/Python resources."""
    from embodied.core.dependencies import dependency_root
    return dependency_root("x5-official") / "py/arx_x5_python"


# Deployment paths used by the X5 console and legacy workflow.
SOURCE_ROOT = PROJECT_ROOT / "src"
ASSETS_ROOT = PROJECT_ROOT / "assets" / "arx_motion"
CALIBRATIONS_ROOT = PROJECT_ROOT / "outputs" / "calibrations" / "arx_motion"
CONFIG_ROOT = PROJECT_ROOT / "configs" / "arx_motion"
WORK_ROOT = Path.home() / "work"
DATASETS_ROOT = WORK_ROOT / "data" / "arx5"
PRE_WEIGHTS_ROOT = WORK_ROOT / "pre_weights"
OUTPUTS_ROOT = PROJECT_ROOT / "outputs" / "arx_motion"
RUNTIME_ROOT = PROJECT_ROOT / "runtime" / "arx_motion"
WEIGHTS_ROOT = ASSETS_ROOT / "weights"
FRONTEND_ROOT = PROJECT_ROOT / "apps" / "console"
HANDEYE_ROOT = SOURCE_ROOT / "legacy" / "hand_eye"
