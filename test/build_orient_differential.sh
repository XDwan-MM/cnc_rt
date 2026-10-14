#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REFERENCE="${1:-$ROOT/../cnc_rt-feature}"
HAL_ROOT="${2:-$ROOT/../HAL}"
mkdir -p "$ROOT/build"
read -r -a qt_cflags <<< "$(pkg-config --cflags Qt5Core)"
flags=(-std=gnu11 -DSHM_STATIC -fPIC -g -O0 -Wall -Wextra -ffunction-sections -fdata-sections)
cc "${flags[@]}" "${qt_cflags[@]}" \
    -I "$ROOT/lib/cncshm/api" -I "$REFERENCE/src/hal" -I "$REFERENCE/src/master" \
    -I /opt/GreeMaster/include/libGREEMASTER \
    -DREFERENCE_DRIVER="\"$REFERENCE/src/hal/driver.c\"" \
    "$ROOT/test/reference_orient_audit.c" -Wl,--gc-sections -o "$ROOT/build/reference_orient_audit"
cc "${flags[@]}" -I "$HAL_ROOT/include" -I "$HAL_ROOT/src" -I "$HAL_ROOT/src/Greemaster" \
    -DHAL_CONTEXT_TEST="\"$HAL_ROOT/test/context_regression.c\"" \
    "$ROOT/test/hal_orient_audit.c" "$HAL_ROOT/src/internal/hal_context.c" \
    "$HAL_ROOT/src/internal/hal_config.c" -Wl,--gc-sections -Wl,--wrap=calloc -lm -pthread \
    -o "$ROOT/build/hal_orient_audit"
"$ROOT/build/reference_orient_audit"
"$ROOT/build/hal_orient_audit"
