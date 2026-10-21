#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
out="${TMPDIR:-/tmp}/rt_hal_sim_sdo_check"
cc -std=c11 -Wall -Wextra -Werror -pthread \
    -I "$root/src/hal" -I "$root/lib/cnc_hal/include" \
    -I "$root/lib/cnc_hal/internal" \
    "$root/test/rt_hal_sim_sdo_check.c" \
    "$root/src/hal/rt_hal_sim_backend.c" \
    "$root/lib/cnc_hal/lib/libcnc_hal.a" -lm -o "$out"
"$out"
