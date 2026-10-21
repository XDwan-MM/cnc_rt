#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
out="$root/build/rt_hal_gree4_binding_check"
cc -std=c11 -O0 -g -Wall -Wextra \
    -I "$root/src/hal" -I "$root/lib/cnc_hal/include" \
    -I "$root/lib/cnc_hal/internal" \
    "$root/test/rt_hal_gree4_binding_check.c" \
    "$root/src/hal/rt_hal_config.c" \
    "$root/src/hal/rt_hal_sim_backend.c" \
    "$root/lib/cnc_hal/lib/libcnc_hal.a" -lm -lpthread -o "$out"
CNC_RT_SIM_GREE4=0 "$out"
CNC_RT_SIM_GREE4=1 "$out"
