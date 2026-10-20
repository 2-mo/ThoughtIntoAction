#!/usr/bin/env bash

# Shared lifecycle helpers for the multi-process web launchers.

web_process_running() {
  local pid="${1:-}"
  local state
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
  state="${state//[[:space:]]/}"
  [[ -n "$state" && "${state:0:1}" != "Z" ]]
}

web_signal_process_group() {
  local signal_name="$1"
  local pid="$2"
  local process_group
  web_process_running "$pid" || return 0
  process_group="$(ps -o pgid= -p "$pid" 2>/dev/null || true)"
  process_group="${process_group//[[:space:]]/}"
  if [[ "$process_group" =~ ^[0-9]+$ && "$process_group" == "$pid" ]]; then
    kill "-$signal_name" -- "-$process_group" 2>/dev/null || true
  else
    kill "-$signal_name" "$pid" 2>/dev/null || true
  fi
}

web_stop_process_groups() {
  local timeout_s="$1"
  shift
  local entry name pid deadline any_running
  local -a entries=("$@")

  for entry in "${entries[@]}"; do
    pid="${entry#*:}"
    web_signal_process_group TERM "$pid"
  done

  deadline=$((SECONDS + timeout_s))
  while ((SECONDS < deadline)); do
    any_running=0
    for entry in "${entries[@]}"; do
      pid="${entry#*:}"
      if web_process_running "$pid"; then
        any_running=1
        break
      fi
    done
    ((any_running == 0)) && break
    sleep 0.1
  done

  for entry in "${entries[@]}"; do
    name="${entry%%:*}"
    pid="${entry#*:}"
    if web_process_running "$pid"; then
      echo "${name} 服务未在 ${timeout_s} 秒内退出；正在结束孤立进程组（PID $pid）。" >&2
      web_signal_process_group KILL "$pid"
    fi
  done

  deadline=$((SECONDS + 3))
  while ((SECONDS < deadline)); do
    any_running=0
    for entry in "${entries[@]}"; do
      pid="${entry#*:}"
      if web_process_running "$pid"; then
        any_running=1
        break
      fi
    done
    ((any_running == 0)) && break
    sleep 0.1
  done

  for entry in "${entries[@]}"; do
    name="${entry%%:*}"
    pid="${entry#*:}"
    if web_process_running "$pid"; then
      echo "${name} 服务进程仍未结束（PID $pid）；请检查系统进程状态。" >&2
    elif [[ "$pid" =~ ^[0-9]+$ ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}
