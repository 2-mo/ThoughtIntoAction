#!/usr/bin/env bash

frontend_dist_dir() {
  local root_dir="$1"
  local frontend_dir="$root_dir/apps/console"
  local dist_dir="$root_dir/apps/console/dist"
  local index_file="$dist_dir/index.html"
  local needs_build=0
  if [[ ! -f "$index_file" ]]; then
    needs_build=1
  elif IFS= read -r _ < <(find \
    "$frontend_dir/src" \
    "$frontend_dir/public" \
    "$frontend_dir/package.json" \
    "$frontend_dir/vite.config.js" \
    -type f -newer "$index_file" -print -quit 2>/dev/null); then
    needs_build=1
  fi
  if ((needs_build)); then
    if ! command -v npm >/dev/null 2>&1; then
      echo "前端源码已更新，但找不到 npm，无法重建：$dist_dir" >&2
      return 1
    fi
    echo "前端源码比构建产物新，正在自动重建……" >&2
    (cd "$frontend_dir" && npm run build >&2)
  fi
  printf '%s\n' "$dist_dir"
}
