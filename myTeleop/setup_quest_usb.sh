#!/usr/bin/env bash
set -Eeuo pipefail

readonly XR_PORT="63901"

if ! command -v adb >/dev/null 2>&1; then
    printf 'adb is not installed.\n' >&2
    exit 1
fi

device_count="$(adb devices | awk 'NR > 1 && $2 == "device" {count++} END {print count+0}')"
if [[ "${device_count}" -ne 1 ]]; then
    printf 'Expected exactly one authorized Quest, found %s.\n' "${device_count}" >&2
    adb devices -l >&2
    exit 1
fi

adb reverse --remove "tcp:${XR_PORT}" >/dev/null 2>&1 || true
adb reverse "tcp:${XR_PORT}" "tcp:${XR_PORT}"

printf 'Quest USB transport is ready: Quest 127.0.0.1:%s -> PC 127.0.0.1:%s\n' \
    "${XR_PORT}" "${XR_PORT}"
printf 'Set the XRoboToolkit server IP in Quest to 127.0.0.1, then press Connect.\n'
