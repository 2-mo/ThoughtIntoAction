#!/usr/bin/env bash
# Real arm on the arx-televr kernel with upstream controls (todo K-08). Started by the operator.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cat <<'TEXT'
真机 · arx 内核（上游 arx-televr 操控，待验收）
  - 启动会使能电机并回 Home（control/config/robot.json 的 park_q）。
  - Ctrl+C 退出进入 SDK 阻尼，不回 Home；退出前先支撑机械臂。
  - 右手：A 按住至少 150 ms 后松开切换模式，A+B 回 Home，Grip 跟随，Trigger 夹爪。
  - 旧机械臂服务仍占用 CAN 时内核拒绝启动：先执行 ./run.sh real shutdown。
  - 验收步骤见 docs/guides/real-arm-acceptance.md。
TEXT
if [[ "${1:-}" == "--yes" ]]; then
  shift
else
  read -r -p "机械臂周围安全、急停在手边，开始？[y/N] " answer
  [[ "$answer" == [yY] ]] || { echo "未启动。"; exit 1; }
fi
"$ROOT/run.sh" quest-transport prepare
"$ROOT/run.sh" can-up
exec "$ROOT/run.sh" control start --backend real --input quest --telemetry "$@"
