#!/usr/bin/env bash
# Real arm on the arx kernel, after a safety confirmation (todo K-08).
#   kernel_real.sh [--yes]                     upstream Quest control only (no command gateway)
#   kernel_real.sh --profile real-quest [--yes] Quest plus platform clients (web console, motion, calibration, policy)
#   kernel_real.sh --profile real-manual [--yes] platform clients only, no Quest
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PROFILE="" YES=0 ARGS=()
while (($#)); do
  case "$1" in
    --profile) PROFILE="$2"; shift 2 ;;
    --yes) YES=1; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
if [[ "$PROFILE" == real-manual ]]; then
  echo 'robot · 真机原位保持'
  echo '启动后使能电机，保持当前位置和夹爪；网页自动接收反馈。'
elif [[ -n "$PROFILE" ]]; then
  echo "robot · 真机配置 $PROFILE"
  echo '启动后使能电机并先保持当前位置，随后按所选输入控制。'
else
  echo 'robot · Quest 原生遥操'
  echo '启动后使能电机、回 Home 并打开夹爪；Grip 跟随，Trigger 控制夹爪。'
fi
echo 'Ctrl+C 退出进入阻尼，退出前请支撑机械臂。'
if ((YES == 0)); then
  read -r -p "准备好后输入 y 启动 robot：[y/N] " answer
  [[ "$answer" == [yY] ]] || { echo "未启动。"; exit 0; }
fi
if [[ -z "$PROFILE" || "$PROFILE" == *quest* ]]; then
  "$ROOT/run.sh" quest-transport prepare
fi
"$ROOT/run.sh" can-up
if [[ -n "$PROFILE" ]]; then
  exec "$ROOT/run.sh" kernel profiles start --profile "$PROFILE" "${ARGS[@]}"
fi
exec "$ROOT/run.sh" control start --backend real --input quest --telemetry "${ARGS[@]}"
