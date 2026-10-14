#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/build"
read -r -a flags <<< "$(pkg-config --cflags --libs Qt5Core)"
g++ -std=c++14 -fPIC -Wall -Wextra -Werror \
    -I "$ROOT/lib/cncshm/api" -I "$ROOT/lib/cncshm/private" \
    "$ROOT/test/rt_site_snapshot.cpp" "${flags[@]}" -o "$ROOT/build/rt_site_snapshot"
"$ROOT/build/rt_site_snapshot" --self-test "$ROOT/test/capture_site.sh"
