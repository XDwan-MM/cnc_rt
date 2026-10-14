#!/bin/bash
# Read-only machine collection. Does not launch/stop RT or write CNC registers.
set -euo pipefail
if (( $# < 2 || $# > 4 )); then
    printf 'Usage: bash capture_site.sh FIRMWARE_DIR NEW_OUTPUT_DIR [RT_PID] [LOG_DIR]\n' >&2
    exit 2
fi
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE="$(realpath -e -- "$1")"
OUT="$(realpath -m -- "$2")"
[[ -d "$FIRMWARE" && ! -e "$OUT" ]] || { echo 'Firmware missing or output exists' >&2; exit 2; }
if [[ "$OUT/" == "$FIRMWARE/"* ]]; then
    echo 'Put the output outside the firmware directory' >&2
    exit 2
fi
SNAPSHOT="$HERE/rt_site_snapshot"
[[ -x "$SNAPSHOT" ]] || SNAPSHOT="$HERE/../build/rt_site_snapshot"
[[ -x "$SNAPSHOT" ]] || { echo 'Missing prebuilt rt_site_snapshot' >&2; exit 2; }
PID="${3:-}"
if [[ -z "$PID" ]]; then
    mapfile -t candidates < <(pgrep -x 'cnc_rt|cnc_rt_d' || true)
    (( ${#candidates[@]} == 1 )) || { echo 'Specify the running RT PID explicitly' >&2; exit 2; }
    PID="${candidates[0]}"
fi
[[ "$PID" =~ ^[0-9]+$ && -r "/proc/$PID/exe" ]] || { echo 'Invalid/inaccessible RT PID' >&2; exit 2; }
mkdir -p -- "$(dirname -- "$OUT")"
mkdir -- "$OUT"
mkdir -- "$OUT/firmware" "$OUT/process"
date -u '+%Y-%m-%dT%H:%M:%SZ' > "$OUT/captured-utc.txt"
uname -a > "$OUT/system.txt"
printf '%s\n' "$PID" > "$OUT/process/pid.txt"
readlink -- "/proc/$PID/exe" > "$OUT/process/exe-path.txt"
readlink -- "/proc/$PID/cwd" > "$OUT/process/cwd.txt"
cp -- "/proc/$PID/exe" "$OUT/process/rt-running.bin"
cp -- "/proc/$PID/maps" "$OUT/process/maps.txt"
tr '\0' '\n' < "/proc/$PID/cmdline" > "$OUT/process/cmdline.txt"
KEY="${CNC_SHM_KEY:-NcData}"
DICT=""
if [[ -r "/proc/$PID/environ" ]]; then
    while IFS= read -r -d '' entry; do
        case "$entry" in
            CNC_SHM_KEY=*) KEY="${entry#*=}" ;;
            CNC_HAL_DEVICES_JSON=*) DICT="${entry#*=}" ;;
        esac
    done < "/proc/$PID/environ"
fi
if [[ -n "$DICT" ]]; then
    [[ "$DICT" = /* ]] || DICT="$(readlink -- "/proc/$PID/cwd")/$DICT"
    if [[ -r "$DICT" ]]; then
        cp -- "$DICT" "$OUT/process/devices.json"
        printf '%s\n' "$DICT" > "$OUT/process/devices-path.txt"
    else
        printf 'Cannot read device dictionary: %s\n' "$DICT" >> "$OUT/missing.txt"
    fi
fi
mkdir -- "$OUT/process/libraries"
while read -r address permissions offset device inode pathname; do
    case "$pathname" in
        */libGREEMASTER.so*)
            pathname="${pathname% (deleted)}"
            if [[ -r "$pathname" ]]; then
                cp -L -- "$pathname" "$OUT/process/libraries/$(basename -- "$pathname")"
            else
                printf 'Cannot read loaded library: %s\n' "$pathname" >> "$OUT/missing.txt"
            fi
            ;;
    esac
done < "$OUT/process/maps.txt"
for part in plc conf data version.sh; do
    if [[ -e "$FIRMWARE/$part" ]]; then
        cp -a -- "$FIRMWARE/$part" "$OUT/firmware/"
    else
        printf 'Missing firmware item: %s\n' "$part" >> "$OUT/missing.txt"
    fi
done
if [[ $# == 4 ]]; then
    LOGS="$(realpath -e -- "$4")"
    [[ -d "$LOGS" && "$OUT/" != "$LOGS/"* ]] || { echo 'Invalid log directory' >&2; exit 2; }
    cp -a -- "$LOGS" "$OUT/logs"
fi
rc=0
"$SNAPSHOT" "$OUT/live" "$KEY" > "$OUT/snapshot-tool.log" 2>&1 || rc=$?
(
    cd -- "$OUT"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 -r sha256sum > SHA256SUMS
)
if (( rc )); then
    printf 'PARTIAL capture: live snapshot failed, retain directory and read %s\n' "$OUT/snapshot-tool.log" >&2
    exit "$rc"
fi
printf 'Captured: %s\nNo CNC state was modified. Keep this entire directory.\n' "$OUT"
