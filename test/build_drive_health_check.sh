#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flags=()
output="$ROOT/build/rt_drive_health_check"
case "${1:-production}" in
    production) ;;
    debug) flags+=(-DDEBUG_MODE_ENABLED); output+="_debug" ;;
    *) echo "usage: $0 [production|debug]" >&2; exit 2 ;;
esac
includes=(-I /opt/GreeMaster/include/libGREEMASTER)
while IFS= read -r dir; do includes+=(-I "$dir"); done < <(find "$ROOT/src" -type d)
for dir in lib/cncshm/api lib/cncshm lib/parser/api lib/parser lib/cnc_hal/include lib/kinematic/include; do
    includes+=(-I "$ROOT/$dir")
done
mkdir -p "$ROOT/build"
if command -v pkg-config >/dev/null; then
    read -r -a qt_cflags <<< "$(pkg-config --cflags Qt5Core)"
else
    qt_headers="$(qmake -query QT_INSTALL_HEADERS)"
    qt_cflags=(-I "$qt_headers" -I "$qt_headers/QtCore")
fi
cc -std=gnu11 -DSHM_STATIC -fPIC "${qt_cflags[@]}" "${flags[@]}" -g -O0 -Wall -Wextra \
    -ffunction-sections -fdata-sections "${includes[@]}" \
    "$ROOT/test/rt_drive_health_check.c" -Wl,--gc-sections -lm \
    -o "$output"
"$output"
