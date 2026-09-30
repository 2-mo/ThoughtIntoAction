import ast
import sys
from pathlib import Path

ROOT = Path(__file__).parents[3]
CORE = ROOT / "src/robot_teleop/core"
RUNTIME = ROOT / "src/robot_teleop/runtime"
FORBIDDEN = {
    "arx5",
    "fastapi",
    "numpy",
    "pathlib",
    "pinocchio",
    "ros",
    "ruckig",
    "rclpy",
    "socket",
    "websockets",
    "yaml",
}


def test_core_imports_only_standard_library_and_itself() -> None:
    violations: list[str] = []
    for path in sorted(CORE.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.partition(".")[0]
                    if root not in sys.stdlib_module_names:
                        violations.append(f"{path.name}: import {alias.name}")
            elif isinstance(node, ast.ImportFrom):
                if node.level == 1:
                    continue
                module = node.module or ""
                root = module.partition(".")[0]
                if node.level > 1 or (
                    root not in sys.stdlib_module_names
                    and not module.startswith("robot_teleop.core")
                ):
                    violations.append(f"{path.name}: from {'.' * node.level}{module}")

    assert violations == []


def test_core_has_no_explicit_forbidden_dependency_or_source_branch() -> None:
    source = "\n".join(
        path.read_text(encoding="utf-8") for path in sorted(CORE.glob("*.py"))
    )

    for dependency in FORBIDDEN:
        assert f"import {dependency}" not in source
    assert 'source_name == "iphone"' not in source
    assert "source_name == 'iphone'" not in source


def test_runtime_does_not_import_offline_kinematics_tools() -> None:
    source = "\n".join(
        path.read_text(encoding="utf-8") for path in sorted(RUNTIME.glob("*.py"))
    )

    assert "robot_teleop.kinematics" not in source
