#!/usr/bin/env bash
# Serve the repository so apps/site/ and its links into docs/ open locally.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="$(PYTHONPATH="$ROOT/src" "${QUEST_BOOTSTRAP_PYTHON:-/usr/bin/python3}" -B -c "from embodied.core.project import port; print(port('site'))")"
echo "宣传网站：http://127.0.0.1:$PORT/apps/site/   Ctrl+C 停止"
exec "${QUEST_BOOTSTRAP_PYTHON:-/usr/bin/python3}" -m http.server "$PORT" --bind 127.0.0.1 --directory "$ROOT"
