#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
includes=()
while IFS= read -r dir; do includes+=(-I "$dir"); done < <(find "$ROOT/src" -type d)
for dir in lib/cncshm/api lib/cncshm lib/parser/api lib/parser lib/cnc_hal/include lib/kinematic/include; do
    includes+=(-I "$ROOT/$dir")
done
mkdir -p "$ROOT/build"
read -r -a qt_cflags <<< "$(pkg-config --cflags Qt5Core)"
cc -std=gnu11 -DSHM_STATIC -fPIC "${qt_cflags[@]}" -g -O0 -Wall -Wextra -ffunction-sections -fdata-sections \
    "${includes[@]}" "$ROOT/test/rt_task_spl_event_check.c" \
    -Wl,--gc-sections -lm -o "$ROOT/build/rt_task_spl_event_check"
"$ROOT/build/rt_task_spl_event_check"