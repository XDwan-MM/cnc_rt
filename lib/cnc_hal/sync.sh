#!/bin/bash
# 从 HAL 工程同步 vendored 副本。
#
# 为什么是"副本"而不是外部路径引用：与 lib/parser、lib/kinematic、lib/CNC_ErrorComp
# 一致的约定 —— 依赖随工程走，构建不依赖某个开发者的目录布局，也不依赖环境变量。
# 代价是 HAL 改了之后要重跑本脚本，所以它把来源（提交号 + ABI 版本）记在 SYNCED.txt 里，
# 一眼就能看出副本旧没旧。
#
# 用法：  ./lib/cnc_hal/sync.sh [HAL 工程根目录]
#         HAL_ROOT=/path/to/HAL ./lib/cnc_hal/sync.sh
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
HAL_ROOT="${1:-${HAL_ROOT:-$(dirname -- "$RT_ROOT")/HAL}}"

HAL_INC="$HAL_ROOT/include"
HAL_SRC="$HAL_ROOT/src"
HAL_LIB="$HAL_ROOT/build/all/libcnc_hal.a"

[[ -f "$HAL_INC/hal_types.h" ]] || { echo "找不到 HAL 公共头：$HAL_INC" >&2; exit 1; }
[[ -f "$HAL_LIB" ]] || { echo "找不到 HAL 静态库：$HAL_LIB（先在 HAL 里跑 ./build_all.sh）" >&2; exit 1; }

# ★ 静态库必须比 HAL 源新，否则拷过来的库和头文件 ABI 对不上。
# 2026-10-08 踩过：库里编译期写死的 abi/struct_size 是旧值（5.0/3160），
# 头文件已是新值（5.1/3168），hal_config_validate 直接拒掉整个配置，
# 离线校验 45 项里错 13 项，真机连启动都过不去。这类错很难从现象反推。
STALE=0
while IFS= read -r f; do
    [[ "$f" -nt "$HAL_LIB" ]] && STALE=1
done < <(find "$HAL_INC" "$HAL_SRC" -name '*.h' -o -name '*.c' 2>/dev/null)
if [[ "$STALE" == "1" ]]; then
    echo "错误：$HAL_LIB 比 HAL 的源码旧——先重建 HAL（./build_all.sh）再同步。" >&2
    echo "      否则库里的 ABI/struct_size 与头文件不一致，配置校验会整体失败。" >&2
    exit 1
fi

echo "HAL 源：$HAL_ROOT"

# 1) 公共头 —— RT 业务代码只看这一层
rm -rf "$SCRIPT_DIR/include"; mkdir -p "$SCRIPT_DIR/include"
cp "$HAL_INC"/*.h "$SCRIPT_DIR/include/"

# 2) 内部头 —— **只给调试构建的驱动层桩用**（rt_hal_sim_backend.c 要扮演驱动层，
#    必须看到 DeviceSlot / DevDictRole / Ds402Request 这些类型）。
#    实时构建的 INCLUDEPATH 里**没有**这个目录，依赖边界不被破坏。
#    保留 Greemaster/ 与 common/ 两级子目录：HAL 里就是按 "Greemaster/device.h"
#    这种方式 include 的，拍平了会全断。
rm -rf "$SCRIPT_DIR/internal"; mkdir -p "$SCRIPT_DIR/internal/Greemaster" "$SCRIPT_DIR/internal/common"
cp "$HAL_SRC"/Greemaster/*.h "$SCRIPT_DIR/internal/Greemaster/"
cp "$HAL_SRC"/common/*.h     "$SCRIPT_DIR/internal/common/"

# 3) 静态库 + 设备字典
rm -rf "$SCRIPT_DIR/lib"; mkdir -p "$SCRIPT_DIR/lib"
cp "$HAL_LIB" "$SCRIPT_DIR/lib/"

# 设备字典是 HAL 的运行时数据，必须跟头和库同版本走。放进 lib/ 之后由
# run.sh 通过 CNC_HAL_DEVICES_JSON 指过去——发布版的编译期路径写死在
# /usr/local/share（在 main_demo.c.o 里），RT 不编译 HAL 源码，改不了它，
# 所以运行时覆盖是唯一的途径。
[[ -f "$HAL_SRC/Greemaster/devices.json" ]] || { echo "找不到设备字典：$HAL_SRC/Greemaster/devices.json" >&2; exit 1; }
cp "$HAL_SRC/Greemaster/devices.json" "$SCRIPT_DIR/lib/devices.json"

# 4) 记录来源，方便判断副本新旧
{
    echo "同步自   : $HAL_ROOT"
    echo "提交     : $(git -C "$HAL_ROOT" rev-parse --short HEAD 2>/dev/null || echo '(非 git 仓库)')"
    echo "提交说明 : $(git -C "$HAL_ROOT" log -1 --format=%s 2>/dev/null || echo '-')"
    echo "同步时间 : $(date -Iseconds)"
    echo "ABI      : $(grep -oP 'HAL_C_ABI_MAJOR\s+\K[0-9]+' "$HAL_INC/hal_types.h")u.$(grep -oP 'HAL_C_ABI_MINOR\s+\K[0-9]+' "$HAL_INC/hal_types.h")u"
    echo "公共头   : $(ls "$SCRIPT_DIR/include" | wc -l) 个"
    echo "内部头   : $(ls "$SCRIPT_DIR/internal/Greemaster" "$SCRIPT_DIR/internal/common" | grep -c '\.h$') 个"
    echo "设备字典 : devices.json sha256=$(sha256sum "$SCRIPT_DIR/lib/devices.json" | cut -c1-16)"
} > "$SCRIPT_DIR/SYNCED.txt"

cat "$SCRIPT_DIR/SYNCED.txt"
echo
echo "完成。如需提交： git add lib/cnc_hal && git commit"
