"""Build the pinned PyPI SDK source with startup gripper turn compensation.

Requires pybind11 and extracted Eigen/spdlog/fmt/KDL development headers in
outputs/build/arx-gripper/deps/usr/include. Links the installed wheel libraries.
Does not import a controller or access CAN.
"""
from pathlib import Path
import argparse
import shutil
import subprocess
import sys
import sysconfig

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "outputs/build/arx-gripper"
SOURCE = BUILD / "source"


def replace(file, old, new):
    path = SOURCE / file
    text = path.read_text()
    if old not in text:
        raise RuntimeError(f"SDK source mismatch: {file}")
    path.write_text(text.replace(old, new))


def main():
    global BUILD, SOURCE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, default=BUILD)
    parser.add_argument('--source-archive', type=Path, help='Pinned git archive supplied by the repository')
    parser.add_argument('--headers', type=Path, help='Extracted development include directory')
    parser.add_argument('--patch-header', type=Path, default=ROOT / 'src/legacy/robot_control/gripper_turns.h')
    args = parser.parse_args()
    BUILD = args.build_dir.resolve()
    SOURCE = BUILD / 'source'
    SOURCE.mkdir(parents=True, exist_ok=True)
    archive = args.source_archive.read_bytes() if args.source_archive else subprocess.check_output(
        ["git", "-C", str(ROOT / "third_party/arx5-sdk"), "archive", "8c797d5"])
    subprocess.run(["tar", "-x", "-C", str(SOURCE)], input=archive, check=True)
    shutil.copy2(args.patch_header, SOURCE / "include/app/gripper_turns.h")
    replace("include/app/config.h", "    double gripper_open_readout;",
            "    double gripper_open_readout;\n    bool gripper_startup_turns = false;")
    replace("include/app/controller_base.h", '#include "app/common.h"',
            '#include "app/common.h"\n#include "app/gripper_turns.h"')
    replace("include/app/controller_base.h", "    void init_robot_();",
            "    GripperTurns gripper_turns_;\n    void init_robot_();")
    replace("src/app/controller_base.cpp",
            "    joint_state_.gripper_pos = motor_msg[robot_config_.gripper_motor_id].angle_actual_rad /\n                               robot_config_.gripper_open_readout * robot_config_.gripper_width;",
            """    const double raw_gripper = motor_msg[robot_config_.gripper_motor_id].angle_actual_rad;
    joint_state_.gripper_pos = robot_config_.gripper_startup_turns
        ? gripper_turns_.feedback(raw_gripper, robot_config_.gripper_open_readout, robot_config_.gripper_width)
        : raw_gripper / robot_config_.gripper_open_readout * robot_config_.gripper_width;""")
    replace("src/app/controller_base.cpp",
            "output_joint_cmd_.gripper_pos / robot_config_.gripper_width * robot_config_.gripper_open_readout;",
            """robot_config_.gripper_startup_turns
                ? gripper_turns_.command(output_joint_cmd_.gripper_pos, robot_config_.gripper_open_readout, robot_config_.gripper_width)
                : output_joint_cmd_.gripper_pos / robot_config_.gripper_width * robot_config_.gripper_open_readout;""")
    replace("src/app/joint_controller.cpp", "    can_handle_.reset_zero_readout(robot_config_.gripper_motor_id);",
            "    can_handle_.reset_zero_readout(robot_config_.gripper_motor_id);\n    gripper_turns_ = GripperTurns{};")
    replace("python/arx5_pybind.cpp",
            '        .def_readwrite("gripper_open_readout", &RobotConfig::gripper_open_readout)',
            '        .def_readwrite("gripper_startup_turns", &RobotConfig::gripper_startup_turns)\n'
            '        .def_readwrite("gripper_open_readout", &RobotConfig::gripper_open_readout)')
    replace("python/arx5_pybind.cpp", "PYBIND11_MODULE(arx5_interface, m)\n{",
            '''PYBIND11_MODULE(arx5_interface, m)
{
    m.attr("quest_gripper_turns_version") = 1;
    py::class_<GripperTurns>(m, "_GripperTurns")
        .def(py::init<>()).def("feedback", &GripperTurns::feedback)
        .def("command", &GripperTurns::command).def("offset", &GripperTurns::offset);
''')
    # Parser declarations are unused here; the wheel's solver implements parsing.
    replace("include/app/solver.h", "#include <kdl_parser/kdl_parser.hpp>", "")
    libs = Path(sysconfig.get_paths()["purelib"]) / "arx5_interface.libs"
    includes = subprocess.check_output([sys.executable, "-m", "pybind11", "--includes"], text=True).split()
    output = SOURCE / "python" / ("arx5_interface" + sysconfig.get_config_var("EXT_SUFFIX"))
    temporary = output.with_name(output.name + ".building")
    headers = args.headers.resolve() if args.headers else BUILD / "deps/usr/include"
    if not (headers / 'eigen3/Eigen/Core').is_file():
        raise RuntimeError('Missing SDK development headers; see docs/gripper-startup.md')
    command = ["g++", "-O1", "-shared", "-std=c++17", "-fPIC", "-pthread", "-DFMT_HEADER_ONLY",
               '-DSDK_ROOT="' + str(SOURCE) + '"', *includes,
               "-I" + str(SOURCE / "include"), "-I" + str(headers), "-I" + str(headers / "eigen3"),
               *[str(SOURCE / p) for p in ("python/arx5_pybind.cpp", "src/app/controller_base.cpp",
                   "src/app/joint_controller.cpp", "src/app/cartesian_controller.cpp", "src/utils.cpp")],
               *[str(next(libs.glob(pattern))) for pattern in ("libhardware-*.so", "libsolver-*.so",
                                                               "liborocos-kdl-*.so*")],
               "-Wl,--disable-new-dtags,-rpath," + str(libs), "-o", str(temporary)]
    subprocess.run(command, check=True)
    temporary.replace(output)
    print(output)


if __name__ == "__main__":
    main()
