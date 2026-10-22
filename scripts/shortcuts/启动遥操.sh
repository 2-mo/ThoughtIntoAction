#!/usr/bin/env bash
set -Eeuo pipefail

# 日常遥操参数：直接修改这里即可；也可在命令行临时覆盖。
# 下拨 1 秒切换控制／待机。启动只连接、不发送命令；松开扳机与按键，右摇杆先回中，再持续下拨 1 秒（每次切换后需回中）。
COLLECTION_TASK="X5 遥操采集"  # 按实际任务修改，写入每条 Episode
# base/tool 模式的固定朝向补偿；calibrated 模式使用实测方向，不使用此值。
OPERATOR_YAW_DEG=0  # 恢复 8adcaf4 启动方式：使用 XRoboToolkit 默认方向，不额外旋转
# 旧版 pose 跟随：以腕部 link6 为控制点，旋转时指尖会绕腕部运动。
ROTATION_FRAME="base"  # 使用固定方向映射；每次握 Grip 时记录当前位姿起点
VR_CONTROL_RATE_HZ=90
SCALE_FACTOR=1.0               # 手柄到机械臂末端的位移比例
# 运动参数；桌面安全边界由机械臂服务读取 configs/project.toml 的 arm_safety。
MAX_EEF_LINEAR_SPEED_M_S=0   # 遥操末端平移目标速度，m/s；0 不附加限速
MAX_EEF_ANGULAR_SPEED_RAD_S=0   # 遥操末端旋转目标速度，rad/s；0 不附加限速
MAX_COMMAND_LEAD_RAD=0   # 规划前目标相对反馈最大超前量；0 关闭此附加限制
PREVIEW_TIME_S=0.05         # Home / 预抓取 SDK 插值目标时间，秒
TELEOP_PREVIEW_TIME_S=0.01  # 遥操 SDK 插值缩短至 10 ms，减少跟随延迟
JOINT_SOFT_LIMIT_MARGIN_RAD=0.05  # 软件关节边界余量，弧度
MAX_JOINT_SPEED_RAD_S=0     # 关节速度请求上限，单位 rad/s
# 速度 0 使用 SDK 各关节硬件上限；正值取此值与硬件上限的较小值。
# 服务层 Ruckig 统一规划速度、加速度和 jerk；以下为待真机验证的初值。
MAX_JOINT_ACCELERATION_RAD_S2=12.0
MAX_JOINT_JERK_RAD_S3=120.0
# Home 按关节行程分两段，中点减速停稳后进入后半段。
HOME_JERK_RAD_S3=16.0
PRESET_JERK_RAD_S3=24.0
HOME_SPEED_RAD_S=2.0
HOME_FINAL_SPEED_RAD_S=0.75
HOME_ACCELERATION_RAD_S2=2.0
PRESET_SPEED_RAD_S=2.0  # 预抓取关节速度上限
PRESET_ACCELERATION_RAD_S2=6.0
# 关节速度、超前量和预览时间也影响预抓取与回零。
# 以上参数覆盖当前控制会话的服务默认值。

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
TELEOP_ARGS=(--profile presets teleop-presets-x5 --actuate --enable-session
    --control-link link6 --control-mode pose
    --collection-task "$COLLECTION_TASK"
    --max-joint-acceleration-rad-s2 "$MAX_JOINT_ACCELERATION_RAD_S2"
    --max-joint-jerk-rad-s3 "$MAX_JOINT_JERK_RAD_S3"
    --home-jerk-rad-s3 "$HOME_JERK_RAD_S3"
    --preset-jerk-rad-s3 "$PRESET_JERK_RAD_S3"
    --home-speed-rad-s "$HOME_SPEED_RAD_S"
    --home-final-speed-rad-s "$HOME_FINAL_SPEED_RAD_S"
    --home-acceleration-rad-s2 "$HOME_ACCELERATION_RAD_S2"
    --preset-speed-rad-s "$PRESET_SPEED_RAD_S"
    --preset-acceleration-rad-s2 "$PRESET_ACCELERATION_RAD_S2"
    --control-rate-hz "$VR_CONTROL_RATE_HZ"
    --scale-factor "$SCALE_FACTOR"
    --operator-yaw-deg "$OPERATOR_YAW_DEG"
    --rotation-frame "$ROTATION_FRAME"
    --max-eef-linear-speed-m-s "$MAX_EEF_LINEAR_SPEED_M_S"
    --max-eef-angular-speed-rad-s "$MAX_EEF_ANGULAR_SPEED_RAD_S"
    --max-joint-speed-rad-s "$MAX_JOINT_SPEED_RAD_S"
    --max-command-lead-rad "$MAX_COMMAND_LEAD_RAD"
    --preview-time-s "$PREVIEW_TIME_S"
    --teleop-preview-time-s "$TELEOP_PREVIEW_TIME_S"
    --joint-soft-limit-margin-rad "$JOINT_SOFT_LIMIT_MARGIN_RAD"
    "$@")
# Showing help must not wake the headset or start an application.
for argument in "$@"; do
    case "$argument" in
        -h|--help)
            exec "$ROOT_DIR/run.sh" "${TELEOP_ARGS[@]}"
            ;;
    esac
done
# 交互终端管理遥操子进程；非交互调用保持前台运行，便于脚本管理。
if [[ -t 0 ]]; then
    exec "$ROOT_DIR/run.sh" quest-console "${TELEOP_ARGS[@]}"
fi
"$ROOT_DIR/run.sh" quest-transport prepare
if ! "$ROOT_DIR/run.sh" quest-wake start; then
    printf 'Quest 唤醒失败，未启动遥操。\n' >&2
    exit 1
fi
"$ROOT_DIR/run.sh" quest-transport wait
exec "$ROOT_DIR/run.sh" "${TELEOP_ARGS[@]}"
