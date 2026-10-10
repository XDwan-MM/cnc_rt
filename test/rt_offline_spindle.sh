#!/bin/bash
# 无界面主轴回归：M3 -> M5 -> M19 -> 释放 -> 再 M19。
#
# 为什么不用界面：RT 会阻塞等 HMI 写 SYS_START_RT（main.cpp 那段 while），
# 所以用 rt_offline_helper 扮演界面。主轴指令走**真路径**——写 G 寄存器制造边沿，
# plc_g_signal_scanner 翻译成主轴事件，再从 F 寄存器读回主轴自己的判断。
#
# 必须整体以 root 跑（cncshm 拒绝非 root 连共享内存）。
# 用法：  sudo ./test/rt_offline_spindle.sh
set -uo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "本脚本必须整体以 root 运行：  sudo $0" >&2
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RT_ROOT="$(dirname -- "$SCRIPT_DIR")"
HELPER="$RT_ROOT/build/rt_spindle_helper"

cd "$RT_ROOT"

# 共享内存库只允许 2 个进程接入，而本回归自己要用满（握手端 + RT）。
# 界面在跑就一定挤掉 RT —— 现象是 RT 直接"共享内存初始化失败"退出，后面每一条
# 断言都 FAIL，看起来像代码坏了。先在这里挡住，别让人白查半天。
HMI_RUNNING=""
for _n in cnc_hmi_v3 cnc_hmi; do   # pgrep -x 一次只收一个模式（给多个会直接报错）
    _p=$(pgrep -x "$_n" 2>/dev/null | tr '\n' ' ' || true)
    if [[ -n "$_p" ]]; then HMI_RUNNING+="$_n($_p) "; fi
done
if [[ -n "$HMI_RUNNING" ]]; then
    echo "检测到界面进程在运行：$HMI_RUNNING" >&2
    echo "  共享内存库只允许 2 个进程接入，本回归要用满 2 个（握手端 + RT）。" >&2
    echo "  界面占一个名额时 RT 必然起不来，后面所有断言都是假 FAIL。" >&2
    echo "  请先停掉界面，再跑本回归。" >&2
    exit 1
fi

# 台账现在只反映"参数域给全了没有"（两个现场确认开关已删），所以直接用
# bin/cnc_rt_d 即可：握手端会把四轴参数写全，台账随之清空，运动请求不再被丢掉。

# 第一个参数是 RT 可执行文件路径（可选）。**不用环境变量**：脚本整体跑在 sudo 下，
# sudo 会把环境清掉，传参才过得去。
RT_BIN="${1:-$RT_ROOT/bin/cnc_rt_d}"
if [[ ! -x "$RT_BIN" ]]; then
    echo "先构建调试版： bash build.sh" >&2
    exit 1
fi

echo "== 0. 清干净环境（残留进程会让握手端加入旧共享内存）=="
pkill -9 -x cnc_rt_d 2>/dev/null
pkill -9 -x cnc_rt   2>/dev/null
sleep 1
for id in $(ipcs -m | awk '$2 ~ /^[0-9]+$/ && $5 >= 10000000 && $6 == 0 {print $2}'); do
    ipcrm -m "$id" 2>/dev/null
done
echo "   清了 $(ipcs -m | awk '$2 ~ /^[0-9]+$/ && $5 >= 10000000 {print $2}' | wc -l) 个残留段"

echo
echo "== 1. 编译握手端 =="
mkdir -p "$RT_ROOT/build"
gcc -o "$HELPER" "$RT_ROOT/test/rt_offline_helper.c" \
    -I lib/cncshm/api -I lib/cncshm \
    -I/usr/include/x86_64-linux-gnu/qt5 -I/usr/include/x86_64-linux-gnu/qt5/QtCore \
    lib/cncshm/bin/libshm.a -lQt5Core -lstdc++ -lpthread -lm || exit 1
echo "   ok"

echo
echo "== 2. 起握手端（无界面）=="
rm -f bin/rt.log
# SPL_S=<rpm> 可覆盖缺省 S 码（500），用来验主轴转速上限那条链：
#   sudo env SPL_S=1500 ./test/rt_offline_spindle.sh
SPL_S="${SPL_S:-}"
HELPER_ARGS=("--spindle")
[[ -n "$SPL_S" ]] && HELPER_ARGS+=("--s" "$SPL_S")
echo "   主轴 S 码：${SPL_S:-缺省 500}"
"$HELPER" "${HELPER_ARGS[@]}" > /tmp/spindle_hs.log 2>&1 &
HS_PID=$!
sleep 2

echo "== 3. 起 RT 调试版（HAL 内存桩）=="
# 只读快照打在定向窗口里（M19 大约在 RT 起来后 12.3 秒 = 第 12300 拍），
# 用来看那时控制字/状态字到底是什么。可用 DUMP_CYCLE 覆盖。
DUMP_CYCLE="${DUMP_CYCLE:-9500,14000}"
echo "   CNC_RT_DUMP=$DUMP_CYCLE"
env CNC_RT_DUMP="$DUMP_CYCLE" "$RT_BIN" > /tmp/spindle_rt.out 2>&1 &
RT_PID=$!

echo "   等握手端跑完整段序列（约 45 秒）..."
set +e
wait "$HS_PID"
HS_RC=$?
set -e
sleep 1
# RT 常常已经收到 SYS_CLOSE_RT 自己退了；这里所有 kill/wait 都必须吞掉非零返回，
# 否则 set -e 会当场把脚本掐掉，后面的断言一段都打不出来。
kill -INT "$RT_PID" 2>/dev/null || true
for _ in $(seq 1 100); do
    kill -0 "$RT_PID" 2>/dev/null || break
    sleep 0.1
done
kill -9 "$RT_PID" 2>/dev/null || true
wait "$RT_PID" 2>/dev/null || true

echo
echo "== 4. 握手端（主轴断言在这里）=="
cat /tmp/spindle_hs.log | sed 's/^/    /'
echo
if [[ "$HS_RC" -eq 0 ]]; then
    echo "   结果：全部 PASS"
else
    echo "   结果：有 FAIL（握手端退出码 $HS_RC）"
fi

echo
echo "== 5. RT 侧证据（关键日志）=="
for pat in "主轴正转事件" "主轴停止事件" "主轴定向事件" "G70.5" "G29.6" "G70.6"; do
    n=$(grep -ac "$pat" /tmp/spindle_rt.out 2>/dev/null || echo 0)
    printf "    %-16s 出现 %s 次\n" "$pat" "$n"
done
echo
echo "---- 定向相关原文 ----"
grep -a "定向" /tmp/spindle_rt.out | tail -12 | sed 's/^/    /' || echo "    （无）"
echo
echo "---- 只读快照（停稳窗口 + 定向窗口）----"
sed -n '/只读快照/,/=====$/p' /tmp/spindle_rt.out | sed 's/^/    /' || echo "    （无）"

echo
echo "---- 力矩状态（主轴停稳后不该再带力矩）----"
# 只在快照确实落在"停稳"窗口（速度=0）时才判；落在减速段就报 N/A ——
# 按本脚本一贯口径，不适用就报 N/A，报成 FAIL 会训练人忽略 FAIL。
check_sp() {
    local cyc="$1" want="$2" desc="$3" line speed got
    line=$(grep -a -A4 "只读快照 @第 ${cyc} 拍" /tmp/spindle_rt.out | grep -a 'HAL轴3' | head -1)
    if [[ -z "$line" ]]; then
        echo "  N/A   $desc（第 ${cyc} 拍没有快照，检查 DUMP_CYCLE）"
        return
    fi
    speed=$(printf '%s' "$line" | grep -aoE '速度=[0-9.]+' | head -1 | cut -d= -f2)
    got=$(printf '%s' "$line" | grep -aoE '使能=-?[0-9]+' | head -1 | cut -d= -f2)
    if [[ "$speed" != "0.000" ]]; then
        echo "  N/A   $desc（第 ${cyc} 拍速度=$speed，不是停稳窗口）"
        return
    fi
    if [[ "$got" == "$want" ]]; then
        echo "  PASS  $desc（使能=$got）"
    else
        echo "  FAIL  $desc（got=${got:-无} want=$want）"
        SP_CHECK_FAIL=1
    fi
}
SP_CHECK_FAIL=0
check_sp 9500  0 "M5 停稳后主轴不带力矩（对齐 v1.27.1）"
check_sp 14000 1 "定向窗口主轴带力矩（对照：证明快照读得到）"

echo
echo "完整输出： /tmp/spindle_rt.out（RT stdout）  bin/rt.log（RT 日志）  /tmp/spindle_hs.log（握手端）"
if [[ "${SP_CHECK_FAIL:-0}" -ne 0 ]]; then
    echo "力矩断言有 FAIL —— 退出码按失败计"
    HS_RC=1
fi
exit "$HS_RC"
