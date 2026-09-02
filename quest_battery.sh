#!/bin/bash

# Quest Pro Battery Monitor
# Headset + Left/Right Touch Pro Controllers

if ! adb get-state >/dev/null 2>&1; then
    echo "Error: Quest is not connected via ADB."
    exit 1
fi

# Headset battery
HEADSET=$(adb shell dumpsys battery 2>/dev/null \
    | awk '/level:/ {print $2; exit}' \
    | tr -d '\r')

# Controller battery information
CONTROLLER_INFO=$(adb shell dumpsys OVRRemoteService 2>/dev/null)

LEFT=$(echo "$CONTROLLER_INFO" \
    | grep "Paired device" \
    | grep "Type: *Left" \
    | sed -n 's/.*Battery:[[:space:]]*\([0-9]*\)%.*/\1/p' \
    | head -1)

RIGHT=$(echo "$CONTROLLER_INFO" \
    | grep "Paired device" \
    | grep "Type: *Right" \
    | sed -n 's/.*Battery:[[:space:]]*\([0-9]*\)%.*/\1/p' \
    | head -1)

[ -z "$HEADSET" ] && HEADSET="N/A"
[ -z "$LEFT" ] && LEFT="N/A"
[ -z "$RIGHT" ] && RIGHT="N/A"

echo "Quest Pro Battery"
echo "======================"
echo "Headset : ${HEADSET}%"
echo "Left    : ${LEFT}%"
echo "Right   : ${RIGHT}%"