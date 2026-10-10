#!/bin/bash
# 编译并运行"轴量按需取用接口"的离线自检：纯换算，不连总线、不跑周期。
# 只编 test/rt_axis_view_check.c（它直接编入 src/utils/rt_axis_view.c，
# 并用一份假的 rt_data 顶掉全局），不链接整个 RT。
#
# 用法：  ./test/build_axis_view_check.sh

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RT_ROOT="$(dirname -- "$SCRIPT_DIR")"
OUT_DIR="$RT_ROOT/build"
OUT="$OUT_DIR/rt_axis_view_check"

mkdir -p "$OUT_DIR"

set -x
cc -std=c11 -g -O0 -Wall -Wextra \
    -I "$RT_ROOT/src/utils" \
    -I "$RT_ROOT/src/datadef" \
    -I "$RT_ROOT/src/hal" \
    -I "$RT_ROOT/lib/cnc_hal/include" \
    -I "$RT_ROOT/lib/cncshm/api" \
    -I "$RT_ROOT/lib/cncshm" \
    -I "$RT_ROOT/lib/parser/api" \
    -I "$RT_ROOT/lib/parser" \
    -o "$OUT" \
    "$SCRIPT_DIR/rt_axis_view_check.c" \
    -lm
set +x

echo
"$OUT"
