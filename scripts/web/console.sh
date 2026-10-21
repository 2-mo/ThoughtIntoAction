#!/usr/bin/env bash
# apps/console on a running arx kernel (todo C-05): the page attaches to the kernel, never starts or stops it.
# Usage: ./run.sh web console [--run control/run/<RUN>] [other embodied.console options]
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="$ROOT/environments/arx_motion/.venv/bin/python"
source "$ROOT/src/legacy/arx_tools/frontend_runtime.sh"
if [[ ! -x "$PYTHON" ]]; then
  echo "缺少控制台环境：./run.sh components setup analysis" >&2
  exit 1
fi
DIST="$(frontend_dist_dir "$ROOT")"
PORT="$(PYTHONPATH="$ROOT/src" "$PYTHON" -B -c "from embodied.core.project import port; print(port('console'))")"
cd "$ROOT"
echo "控制台：http://127.0.0.1:$PORT   内核：先运行 ./run.sh kernel sim 或 ./run.sh kernel real，页面上点“连接”接入"
echo "相机与手势另开：./run.sh camera rgbd、./run.sh web vision；负载 / 夹爪标定与方块任务暂用 ./run.sh web legacy"
exec env PYTHONPATH="$ROOT/src" "$PYTHON" -B -m embodied.console --kernel --port "$PORT" --frontend-directory "$DIST" "$@"
