#!/usr/bin/env bash
set -Eeuo pipefail

# 日常遥操参数：直接修改这里即可；也可在命令行临时覆盖。
COLLECTION_TASK="X5 遥操采集"  # 按实际任务修改，写入每条 Episode
SCALE_FACTOR=1.0               # 手柄到机械臂末端的位移比例
MAX_JOINT_SPEED_RAD_S=2.0     # 关节速度请求上限，单位 rad/s
# 此速度覆盖服务默认值；只有控制端未传速度时才使用 [arm_service] 默认值。

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/." && pwd)"
TELEOP_ARGS=(--profile presets teleop-presets-x5 --actuate --enable-session
    --collection-task "$COLLECTION_TASK"
    --scale-factor "$SCALE_FACTOR"
    --max-joint-speed-rad-s "$MAX_JOINT_SPEED_RAD_S"
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
