#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/build"
read -r -a qt_flags <<< "$(pkg-config --cflags --libs Qt5Core)"
c++ -std=c++17 -DSHM_STATIC -fPIC -Wall -Wextra \
    -I "$ROOT/lib/cncshm/api" -I "$ROOT/lib/cncshm" \
    "$ROOT/test/rt_m29_plc_replay.cpp" "${qt_flags[@]}" \
    -o "$ROOT/build/rt_m29_plc_replay"
"$ROOT/build/rt_m29_plc_replay" "${1:?Pass the firmware ladder JSON path}"
