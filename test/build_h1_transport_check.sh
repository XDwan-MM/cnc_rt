#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root/build/h1-migration"
cc -std=gnu11 -g -O1 -Wall -Wextra -Werror -fno-omit-frame-pointer -fno-pie -no-pie \
    -fsanitize=address,undefined -fno-sanitize-recover=all -ffunction-sections -fdata-sections \
    -I "$root/src/hal" -I "$root/src/threads" -I "$root/lib/cnc_hal/include" \
    -I "$root/lib/cnc_hal/internal" \
    "$root/test/rt_hal_h1_transport_check.c" "$root/src/hal/rt_hal_config.c" "$root/src/threads/rt_sem.c" \
    "$root/lib/cnc_hal/lib/libcnc_hal.a" -Wl,--gc-sections -lm -pthread \
    -o "$root/build/h1-migration/h1_transport_check"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
    "$root/build/h1-migration/h1_transport_check"
