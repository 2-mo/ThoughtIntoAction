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
cat <<'TEXT'
真机 · arx 内核（待验收）
  - 启动会使能电机并回 Home（control/config/robot.json 的 park_q，默认全零）；先扶好机械臂。
  - Ctrl+C 退出进入 SDK 阻尼，不回 Home；退出前先支撑机械臂。
  - Quest 右手：A 按住至少 150 ms 后松开切换模式，A+B 回 Home，Grip 跟随，Trigger 夹爪。
  - 旧机械臂服务仍占用 CAN 时内核拒绝启动：先执行 ./run.sh real shutdown。
  - 验收步骤见 docs/guides/real-arm-acceptance.md。
TEXT
[[ -n "$PROFILE" ]] && echo "  - 模式：$PROFILE（带命令网关：网页控制台、动作、标定、策略可接入）"
if ((YES == 0)); then
  read -r -p "机械臂周围安全、急停在手边，开始？[y/N] " answer
  [[ "$answer" == [yY] ]] || { echo "未启动。"; exit 1; }
fi
if [[ -z "$PROFILE" || "$PROFILE" == *quest* ]]; then
  "$ROOT/run.sh" quest-transport prepare
fi
"$ROOT/run.sh" can-up
if [[ -n "$PROFILE" ]]; then
  exec "$ROOT/run.sh" kernel profiles start --profile "$PROFILE" "${ARGS[@]}"
fi
exec "$ROOT/run.sh" control start --backend real --input quest --telemetry "${ARGS[@]}"
