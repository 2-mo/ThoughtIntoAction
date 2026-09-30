"""Offline robot-model and kinematics adapters outside the canonical core."""

from .checks import KinematicPathReport, KinematicPathStep, KinematicTarget
from .config import KinematicModelConfig, ModelConfigurationError, load_model_config
from .models import (
    IKResult,
    IKSolverConfig,
    JacobianDiagnostics,
    JointLimits,
    KinematicsUnavailableError,
    RobotModel,
)
from .pinocchio import PinocchioKinematics

__all__ = [
    "IKResult",
    "IKSolverConfig",
    "JacobianDiagnostics",
    "JointLimits",
    "KinematicModelConfig",
    "KinematicPathReport",
    "KinematicPathStep",
    "KinematicTarget",
    "KinematicsUnavailableError",
    "ModelConfigurationError",
    "PinocchioKinematics",
    "RobotModel",
    "load_model_config",
]
