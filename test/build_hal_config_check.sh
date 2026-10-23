#!/bin/bash
# 编译并运行台架 profile 的离线校验程序。
#
# 依赖 lib/cnc_hal/ 下的 vendored 副本（与 lib/parser 等同一约定，自包含）。
# HAL 改了之后先跑 lib/cnc_hal/sync.sh 重新同步，再跑本脚本。
#
# 用法：  ./test/build_hal_config_check.sh

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RT_ROOT="$(dirname -- "$SCRIPT_DIR")"
HAL_PKG="$RT_ROOT/lib/cnc_hal"
HAL_INC="$HAL_PKG/include"
HAL_LIB="$HAL_PKG/lib/libcnc_hal.a"
OUT_DIR="$RT_ROOT/build"
OUT="$OUT_DIR/rt_hal_config_check"

if [[ ! -f "$HAL_INC/hal_types.h" || ! -f "$HAL_LIB" ]]; then
    echo "lib/cnc_hal/ 下的 vendored 副本不完整" >&2
    echo "先同步： ./lib/cnc_hal/sync.sh" >&2
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "HAL 副本 = $HAL_PKG"
echo "链接     = $HAL_LIB"
echo

# 只链 HAL 静态库，**不带** -lethercat、不带 RT 的 master.c/hal_driver.c：
# 那三者与 HAL 静态库有 31 个重名符号，同一次链接会互相覆盖。
# 配置层只用到 hal_config_validate，归档里只有 hal_config.o 会被拉进来，
# 所以连 GREEMASTER/xml2 都不需要。
set -x
cc -std=c11 -g -O0 -Wall -Wextra \
    -I "$RT_ROOT/src/hal" \
    -I "$HAL_INC" \
    -o "$OUT" \
    "$SCRIPT_DIR/rt_hal_config_check.c" \
    "$RT_ROOT/src/hal/rt_hal_config.c" \
    "$HAL_LIB"
set +x

echo
echo "---- 运行 ----"
"$OUT"
