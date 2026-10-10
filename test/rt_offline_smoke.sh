#!/bin/bash
# 离线冒烟：不接硬件，验证 cnc_rt 用新 HAL 的启动路径。
#
# 覆盖 B2 的验收项：启动打印 profile、配置校验通过、不使能不发运动、收到关闭信号
# 后干净退出。**用调试构建**——它链接的是 rt_hal_sim_backend 的内存桩，不碰真总线。
#
# 为什么整个脚本要用 sudo 跑：cncshm 拒绝非 root 进程连接共享内存（共享内存库
# 自己的策略，不是本工程的）。脚本内部不再调 sudo——那些内层调用拿不到终端，
# 会以 "a terminal is required to read the password" 失败（实测踩过）。
#
# 用法：  sudo ./test/rt_offline_smoke.sh [保持秒数] [params]
#   params —— 顺带往参数域写一组台架参数，验证 "参数域 → rt_hal_config → HAL 配置"
#             那条链。不加就是"参数域为空"的路径（RT 应正确退回占位值并挂满台账）。
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "本脚本必须整体以 root 运行（共享内存库拒绝非 root 进程）：" >&2
    echo "    sudo $0 $*" >&2
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RT_ROOT="$(dirname -- "$SCRIPT_DIR")"
HOLD="${1:-8}"
PARAMS=0
FAULT=0
BUSFAIL=0
READY=0
ESTOPMID=0
DRVERR=0
DRVERRF=0
for a in "$@"; do
    [[ "$a" == "params" ]]  && PARAMS=1
    [[ "$a" == "fault" ]]   && { FAULT=1; PARAMS=1; }
    [[ "$a" == "busfail" ]] && { BUSFAIL=1; PARAMS=1; }
    # ready：只释放急停、不注入故障。用来证明"轴真的能被使能"——fault 场景
    # 的主轴故障一直不清，报警中不该带力矩，所以它证明不了这条链。
    [[ "$a" == "ready" ]]   && { READY=1; PARAMS=1; }
    # estopmid：**复位在飞时按急停**，验"急停优先于复位"。复用 fault 的故障注入，
    # 但故障要"不消失"（CNC_RT_SIM_FAULT_STUCK），否则复位几拍就收口、按不住窗口。
    [[ "$a" == "estopmid" ]] && { ESTOPMID=1; FAULT=1; PARAMS=1; }
    # drverr：往主轴注入驱动器故障码（0x603F），验"故障码 → is_error → F45.0/F0.6"。
    [[ "$a" == "drverr" ]] && { DRVERR=1; PARAMS=1; }
    # drverrf：故障码只注入 X 轴（从站 3）—— 反向对照，主轴报警位必须仍是 0。
    [[ "$a" == "drverrf" ]] && { DRVERRF=1; PARAMS=1; }
done
# 握手端放在工程的 build/ 下，不放 /tmp：脚本以 root 跑，放 /tmp 会留下 root 所有
# 的文件，下次普通用户再跑就覆盖不了。
HANDSHAKE="$RT_ROOT/build/rt_handshake"

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

# check <说明> <grep 模式> —— 各验收段共用。定义在顶部，别放进某个分支里：
# 非该分支运行时函数没定义，set -e 会让脚本直接中止（踩过）。
check() {
    if grep -q "$2" /tmp/rt.out; then echo "  PASS  $1"
    else echo "  FAIL  $1  （日志里没有 /$2/）"; fi
}

if [[ ! -x bin/cnc_rt_d ]]; then
    echo "先构建调试版： bash build.sh debug" >&2
    exit 1
fi

echo "== 0. 清干净环境（残留进程会让握手端加入旧共享内存）=="
# 共享内存库只允许 2 个进程接入：上一次跑剩下的段/进程不清掉，这次 RT 直接
# "共享内存初始化失败"退出，后面每一条断言都是假 FAIL。spindle 脚本一直有这一步，
# smoke 脚本漏了 —— 补上。（脚本整体以 root 跑，所以 ipcrm 能清掉别人的段。）
pkill -9 -x cnc_rt_d 2>/dev/null || true
pkill -9 -x cnc_rt   2>/dev/null || true
pkill -9 -f rt_handshake 2>/dev/null || true
sleep 1
for id in $(ipcs -m | awk '$2 ~ /^[0-9]+$/ && $5 >= 10000000 {print $2}'); do
    ipcrm -m "$id" 2>/dev/null || true
done
echo "   清了 $(ipcs -m | awk '$2 ~ /^[0-9]+$/ && $5 >= 10000000 {print $2}' | wc -l) 个残留段"
echo

echo "== 1. 编译离线辅助端（替代 Qt 界面）=="
mkdir -p "$RT_ROOT/build"
gcc -o "$HANDSHAKE" "$RT_ROOT/test/rt_offline_helper.c" \
    -I lib/cncshm/api -I lib/cncshm \
    -I/usr/include/x86_64-linux-gnu/qt5 -I/usr/include/x86_64-linux-gnu/qt5/QtCore \
    lib/cncshm/bin/libshm.a -lQt5Core -lstdc++ -lpthread -lm

echo
echo "== 2. 起握手端（后台），再跑 RT 调试版 =="
echo "   退出方式：调试模式下 main.cpp 只等 SIGINT（main_exit_flag），不读"
echo "   SYS_CLOSE_RT——那是旧代码就有的行为，不是本次改动引入的。所以下面用"
echo "   关闭信号之外再补一个 SIGINT，走真正的干净退出路径。"
if [[ "$PARAMS" == "1" ]]; then
    echo "   模式：注入台架参数 —— 期望 profile 里四轴显示 [换算: 已装载] 与真实当量"
else
    echo "   模式：参数域为空 —— 期望四轴显示 [换算: **占位值**] 且台账挂满"
fi
echo "   观察点：启动日志里应有 '==== HAL 台架 profile: bench-6slave rev1 ===='"
echo "           以及未确认台账；不应出现任何使能或运动指令。"
echo
rm -f bin/rt.log
HELPER_ARGS=("$HOLD")
[[ "$PARAMS" == "1" ]] && HELPER_ARGS+=("--params")
[[ "$FAULT"  == "1" ]] && HELPER_ARGS+=("--reset")
[[ "$READY"  == "1" ]] && HELPER_ARGS+=("--reset")   # 同样靠 G8.4 0→1 释放急停
[[ "$ESTOPMID" == "1" ]] && HELPER_ARGS+=("--estop-mid-reset")
[[ "$DRVERR"   == "1" ]] && HELPER_ARGS+=("--drive-err")
[[ "$DRVERRF"  == "1" ]] && HELPER_ARGS+=("--drive-err-feed")
"$HANDSHAKE" "${HELPER_ARGS[@]}" > /tmp/hs.log 2>&1 &
HS_PID=$!
sleep 1

# 不用 timeout 的退出码判成败：timeout 只要超时就返回 124，哪怕进程收到信号后
# 干净退出了（用过 -s INT，实测踩过这个坑）。改成自己发信号、自己等、自己判。
set +e
RT_ENV=("CNC_RT_DUMP=1")   # 让 RT 打一次只读快照，C 阶段验收要看这个
# fault 场景要看"复位收口之后"的状态：辅助端在 RT 起来约 2 秒后才释放急停
# （周期 1ms，约第 2000 拍），第 100 拍时复位根本还没发生。改打第 5000 拍
# （约 5 秒；本场景保持 8 秒）。
[[ "$FAULT"   == "1" ]] && RT_ENV=("CNC_RT_DUMP=5000")
[[ "$READY"   == "1" ]] && RT_ENV=("CNC_RT_DUMP=5000")
[[ "$FAULT"   == "1" ]] && RT_ENV+=("CNC_RT_SIM_FAULT=2")   # 主轴（从站 2）上电即 Fault
# estopmid：故障一直不消失（把复位按在 PENDING，窗口从几拍拉成 5 秒），
# 快照取**按压前/后各一拍**，用来判断急停有没有真的到驱动层。
[[ "$ESTOPMID" == "1" ]] && RT_ENV=("CNC_RT_DUMP=2500,5500" "CNC_RT_SIM_FAULT=2" "CNC_RT_SIM_FAULT_STUCK=1")
[[ "$DRVERR"   == "1" ]] && RT_ENV+=("CNC_RT_SIM_ERR=2,0x1234")   # 主轴（从站 2）报故障码
[[ "$DRVERRF"  == "1" ]] && RT_ENV+=("CNC_RT_SIM_ERR=3,0x1234")   # 只 X 轴（从站 3）报故障码
[[ "$BUSFAIL" == "1" ]] && RT_ENV+=("CNC_RT_SIM_FAIL=1")    # 所有驱动读失败
env "${RT_ENV[@]}" ./bin/cnc_rt_d > /tmp/rt.out 2>&1 &
RT_PID=$!
sleep "$HOLD"
kill -INT "$RT_PID" 2>/dev/null
EXITED=0
for _ in $(seq 1 100); do          # 最多等 10 秒
    kill -0 "$RT_PID" 2>/dev/null || { EXITED=1; break; }
    sleep 0.1
done
if [[ "$EXITED" -eq 1 ]]; then
    wait "$RT_PID"; RT_RC=$?
else
    echo "!! SIGINT 后 10 秒仍未退出，强杀" >&2
    kill -9 "$RT_PID" 2>/dev/null
    wait "$RT_PID" 2>/dev/null
    RT_RC=124
fi
set -e
wait "$HS_PID" 2>/dev/null || true

echo
echo "== 3. 结果 =="
if [[ "$RT_RC" -eq 124 ]]; then
    echo "退出：**SIGINT 后 10 秒未退出，被强杀** —— 这是缺陷"
else
    echo "退出：干净退出（退出码 $RT_RC）"
fi
echo "退出序列（最后几行）:"
tail -6 /tmp/rt.out | sed 's/^/    /' 
echo
echo "---- HAL profile 打印 ----"
grep -A 22 "HAL 台架 profile" /tmp/rt.out || echo "（没打印 profile —— 启动没走到 HAL create）"
echo
echo "---- 未确认台账 ----"
grep -A 10 "未确认台账" /tmp/rt.out || echo "（没打印台账）"
echo
echo "---- 故障复位链 ----"
if [[ "$ESTOPMID" == "1" ]]; then
    # 复位在飞时按急停：期望"急停优先"——放弃复位等待，且急停真的到 HAL。
    check "复位等待被急停放弃（急停优先）" "放弃故障复位等待"
    # 光看"使能位"验不出来：复位等待期整个 poll 都早退，四轴本来就没使能（0 -> 0）。
    # 能区分的是**状态字**（仿真桩按控制字算，见 rt_hal_sim_backend.c:81-86）：
    #   按压前：复位等待期，进给轴是普通去使能        → 0x0040（Switch on disabled）
    #   按压后：急停的控制字 0x07 真下发到驱动        → 0x0023（Switched on）且仍未使能
    # 普通去使能只会给 0x0040/0x0021，给不出 0x0023 —— 这就是"急停到了驱动层"的证据。
    # （管道里的 grep 可能不匹配，加 || true 免得 set -e 把脚本掐掉。）
    pre_sw=$( grep -a -A5 '只读快照 @第 2500 拍' /tmp/rt.out | grep -a 'HAL轴0' | grep -aoE '状态字=0x[0-9A-Fa-f]+' | cut -d= -f2 || true )
    post_sw=$( grep -a -A5 '只读快照 @第 5500 拍' /tmp/rt.out | grep -a 'HAL轴0' | grep -aoE '状态字=0x[0-9A-Fa-f]+' | cut -d= -f2 || true )
    post_en=$( grep -a -A5 '只读快照 @第 5500 拍' /tmp/rt.out | grep -a 'HAL轴0' | grep -aoE '使能=-?[0-9]+' | cut -d= -f2 || true )
    if [[ "$pre_sw" == "0x0040" && "$post_sw" == "0x0023" && "$post_en" == "0" ]]; then
        echo "  PASS  急停到了驱动层：进给轴 状态字 $pre_sw -> $post_sw（Switched on，且未使能）"
    else
        echo "  FAIL  急停没到驱动层：状态字 ${pre_sw:-无} -> ${post_sw:-无}，使能=${post_en:-无}（期望 0x0040 -> 0x0023，使能=0）"
    fi
    # 反向：复位是被"放弃"的，不该收口（既不该报完成，也不该报超时）。
    if grep -aq -e '故障复位完成' -e '故障复位超时' /tmp/rt.out; then
        echo "  FAIL  复位居然收口了（应被急停放弃）"
    else
        echo "  PASS  复位未收口（被急停放弃，符合预期）"
    fi
elif [[ "$FAULT" == "1" ]]; then
    # 逐段断言，别靠肉眼看日志。每一段对应链上的一环，缺哪段就知道断在哪。
    check "模拟后端注入了 Fault"          "注入 Fault 态"
    check "急停 G8.4 上升沿触发了自动复位" "G8.4信号由0变1"
    check "复位事件被任务层处理"          "处理急停复位事件"
    check "handle_alarm_cmd 走到复位分支" "handle_alarm_cmd: 报警已清除"
    check "四轴复位收口且保持未使能"      "故障复位完成，轴保持未使能"
else
    echo "（未启用 fault 模式，跳过）"
fi
echo
echo "---- C 阶段：不使能也能读到正确数据 ----"
# 只在该断言**适用**的模式下断言。不适用时报 N/A ——
# 报成 FAIL 会训练人忽略 FAIL，那比不报还糟。
if [[ "$BUSFAIL" == "1" ]]; then
    echo "  N/A  总线故障模式下周期循环提前退出，没有快照（预期行为）"
elif grep -q "只读快照" /tmp/rt.out; then
    sed -n '/只读快照/,/^\[HAL\] =====/p' /tmp/rt.out | head -10 | sed 's/^/    /'
    # estopmid 场景急停是按住的，所以走下面 else 的"急停未释放"口径。
    if [[ "$READY" == "1" || ( "$FAULT" == "1" && "$ESTOPMID" != "1" ) ]]; then
        # 这两个场景都释放了急停、故障复位也收口了，所以 RT 应按 is_disable 表达
        # "使能"，轴真的走到 OperationEnabled。这是"使能链通了"的端到端证据：
        # 改动前全 RT 没有任何地方申请使能，四轴永远停在 SwitchOnDisabled（0x0040）。
        # ready 更干净（不注入故障），fault 顺带证明"故障复位之后还能重新使能"。
        check "按意图重新使能（使能链通）" "使能意图 -> 使能"
        check "进给轴已使能"               "product=0x9002 | 使能=1"
    else
        # 急停始终未释放（面板输入全零 → G8.4=0）：不允许带力矩。
        # 注意断言要落到具体轴上——早先这里只 grep 一个 "使能=0"，
        # 主轴在 Fault 态也会让整条通过，把三根进给轴被使能的事实盖住。
        check "急停未释放，四轴均不带力矩" "product=0x9002 | 使能=0"
    fi
    # 进给轴的实际速度现在从 HalCAxisStatus.actual_speed 取（HAL ABI 5.1 起）。
    # 本台架模拟后端的进给从站**没有**映射速度 PDO，所以这里应当明确标出"无速度PDO"，
    # 而不是静默显示 0——"读不到"和"停着"必须能区分开，否则进给率会一直假装是 0。
    check "没映射速度 PDO 时明确标注（不是静默的 0）" "(无速度PDO)"
    check "设备身份读到了（绑定与身份链路通）"   "身份=SIM "
    if [[ "$PARAMS" == "1" ]]; then
        # 进给轴位置应等于 -enc_off（计数为 0）：把当量、enc_off、坐标基准这条换算链一次验掉
        check "进给轴坐标 = -enc_off（换算链正确）" "位置=-150.000000"
    else
        echo "  N/A  坐标断言需要参数域有值；本模式参数域为空，位置按占位当量算"
    fi
    check "X 映像里能看到 X8.4 的急停位"        "00 00 00 00 00 00 00 00 10"
else
    echo "  FAIL  没有只读快照（CNC_RT_DUMP 没生效）"
fi
echo
echo "---- 驱动器故障码（0x603F → is_error）----"
if [[ "$DRVERR" == "1" || "$DRVERRF" == "1" ]]; then
    check "驱动器报错进了日志（带轴名与码）" "轴驱动器报错：0x603F="
    # 握手端的断言在 /tmp/hs.log 里；这个脚本默认不打印它，这里显式带出来，
    # 否则"断言过了"只能靠退出码猜。
    grep -a '\[SPL\]' /tmp/hs.log | sed 's/^/    /' || true
    if grep -aq 'FAIL' /tmp/hs.log; then
        echo "  FAIL  握手端有 FAIL（见上面 [SPL] 行）"
    else
        echo "  PASS  握手端断言全过（F45.0 / F0.6）"
    fi
else
    echo "（未启用 drverr / drverrf 模式，跳过）"
fi
echo
echo "---- HAL 错误 → 报警 转换 ----"
if [[ "$BUSFAIL" == "1" ]]; then
    if grep -qE "HAL 错误 rc=[0-9]+" /tmp/rt.out; then
        echo "  PASS  产生了 HAL 错误报警："
        grep -oE "HAL 错误 rc=[0-9]+（[^）]*）-> 报警 [0-9]+" /tmp/rt.out | head -2 | sed 's/^/        /'
    else
        echo "  FAIL  总线故障了但没产生报警（旧路径会变成报警，不能退化成日志）"
    fi
    grep -q "周期循环 失败" /tmp/rt.out && echo "  PASS  周期循环退出原因被记录"
else
    echo "（未启用 busfail 模式，跳过）"
fi
echo
echo "---- 使能/运动痕迹（应为空）----"
grep -iE "使能|enable|write_pos|运动" /tmp/rt.out | grep -v "不使能\|禁止运动\|未确认\|运动控制器\|运动线程" || echo "（无）"
echo
echo "---- 遗留进程检查 ----"
if pgrep -x cnc_rt_d > /dev/null; then
    echo "!! 还有 cnc_rt_d 进程活着"; pgrep -ax cnc_rt_d
else
    echo "无遗留 cnc_rt_d 进程"
fi
echo
echo "完整输出: /tmp/rt.out    握手端输出: /tmp/hs.log    RT 日志: bin/rt.log"
