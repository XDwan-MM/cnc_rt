#!/bin/bash

# ================= 配置区域 =================
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
# 获取所有以 cnc_hmi_ 开头的目录（仅目录，非文件）
# shopt -s nullglob 防止无匹配时返回字面量 "*cnc_hmi_*"
shopt -s nullglob
HMI_DIRS=("$SCRIPT_DIR"/cnc_hmi*/)
RT_DIRS=("$SCRIPT_DIR"/cnc_rt*/)
# 恢复默认设置
shopt -u nullglob
if [ ${#HMI_DIRS[@]} -eq 0 ]; then
    echo "错误: 未找到以 'cnc_hmi' 开头的目录" >&2
    exit 1
fi
if [ ${#RT_DIRS[@]} -eq 0 ]; then
    echo "错误: 未找到以 'cnc_rt' 开头的目录" >&2
    exit 1
fi
# 取第一个匹配的目录
HMI_BIN_DIR="${HMI_DIRS[0]}/bin"
RT_BIN_DIR="${RT_DIRS[0]}/bin"

CMD1="sudo -E LD_LIBRARY_PATH=../include/cncshm/bin ./cnc_hmi_v3"
CMD2="sudo -E LD_LIBRARY_PATH=../lib/cncshm/bin ./rt_cnc_1"
# >> 表示追加写入，> 表示覆盖写入。建议用 >> 避免每次重启清空日志。
# CMD2="sudo -E LD_LIBRARY_PATH=../lib/cncshm/bin ./rt_cnc_1 > cnc_rt.log 2>&1"

TERMINAL_OPTIONS=("gnome-terminal" "xterm" "konsole" "xfce4-terminal" "mate-terminal")
# 注意：把 xterm/konsole 放在前面，它们的 PID 行为更可靠
# ============================================

echo "=========================================="
echo "                 格力数控系统               "
echo "=========================================="

# 1. 查找可用终端
TERMINAL_CMD=""
for term in "${TERMINAL_OPTIONS[@]}"; do
    if command -v "$term" &> /dev/null; then
        TERMINAL_CMD="$term"
        break
    fi
done

if [ -z "$TERMINAL_CMD" ]; then
    echo "[ERROR] 找不到终端模拟器."
    echo "请按照其中一种: ${TERMINAL_OPTIONS[*]}"
    exit 1
fi

echo "[INFO] 使用终端模拟器: $TERMINAL_CMD"
echo "[TIPS] 使用 Ctrl+C 关闭系统"
echo "------------------------------------------"

# 存储实际业务进程的 PID（不是终端模拟器的 PID）
ACTUAL_PIDS=()
PID_FILES=()

# 2. 启动函数
launch_process() {
    local cmd="$1"
    local name="$2"
    local pidfile
    pidfile=$(mktemp /tmp/cnc_${name}_XXXXXX.pid)
    PID_FILES+=("$pidfile")
    
    # 关键修复：在终端里启动子 shell，后台运行真实命令，把它的 PID 写文件
    # 这样 PID 文件里记录的是 "sudo ./rt_cnc_1" 的 PID，而不是终端或 bash 的 PID
    local inner_cmd="($cmd) & echo \$! > $pidfile; wait \$!; echo '[EXIT] $name finished'; read -p 'Press Enter to close...'"
    
    case "$TERMINAL_CMD" in
        xterm)
            xterm -T "$name" -e "bash -c '$inner_cmd'" &
            ;;
        konsole)
            konsole --new-tab -e "bash -c '$inner_cmd'" &
            ;;
        xfce4-terminal)
            xfce4-terminal --title="$name" -e "bash -c '$inner_cmd'" &
            ;;
        mate-terminal)
            mate-terminal --title="$name" -e "bash -c '$inner_cmd'" &
            ;;
        gnome-terminal)
            # gnome-terminal 会立即 fork 退出，所以我们不依赖它的 PID
            gnome-terminal -- bash -c "$inner_cmd" &
            ;;
    esac
    
    local term_pid=$!
    
    # 等待实际进程 PID 写入（最多 5 秒）
    local actual_pid=""
    for i in {1..50}; do
        if [ -s "$pidfile" ]; then
            actual_pid=$(cat "$pidfile")
            rm -f "$pidfile"
            break
        fi
        sleep 0.1
    done
    
    if [ -n "$actual_pid" ]; then
        ACTUAL_PIDS+=("$actual_pid")
        echo "[START] 启动进程: $name PID: $actual_pid"
    else
        echo "[WARN] $name: 无法获取进程PID $term_pid"
    fi
}

# 3. 启动进程
cd "$HMI_BIN_DIR"
[ -n "${CMD1:-}" ] && launch_process "$CMD1" "CNC_HMI"
cd "$RT_BIN_DIR"
[ -n "${CMD2:-}" ] && launch_process "$CMD2" "CNC_RT"

if [ ${#ACTUAL_PIDS[@]} -eq 0 ]; then
    echo "[ERROR] 进程数量0, 找不到启动的进程"
    rm -f "${PID_FILES[@]}"
    exit 1
fi

# 4. 清理函数（关键修复：杀的是实际业务进程，不是终端）
cleanup() {
    echo ""
    echo "------------------------------------------"
    echo "[INFO] 接收到 Ctrl+C 退出信号"
    
    # 先优雅终止实际 CNC 进程
    for pid in "${ACTUAL_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[INFO] 发送退出信号到进程PID $pid..."
            kill -TERM "$pid" 2>/dev/null
        else
            echo "[INFO] 进程PID $pid 已关闭"
        fi
    done
    
    sleep 2
    
    # 强制杀死残留（sudo 启动的进程可能需要 root 权限才能 kill）
    for pid in "${ACTUAL_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[INFO] 强制退出进程PID $pid..."
            kill -9 "$pid" 2>/dev/null || sudo kill -9 "$pid" 2>/dev/null
        fi
    done
    
    # 清理终端残留（通过进程名补刀，防止 gnome-terminal 的孤儿窗口）
    pkill -f "rt_cnc_1" 2>/dev/null
    pkill -f "rt_cnc_2" 2>/dev/null
    
    rm -f "${PID_FILES[@]}"
    echo "[INFO] 所有进程已关闭"
    echo "------------------------------------------"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 5. 主循环：监控实际业务进程
while true; do
    all_dead=true
    for pid in "${ACTUAL_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            all_dead=false
            break
        fi
    done
    
    if [ "$all_dead" = true ]; then
        echo "[INFO] 所有子进程已退出"
        break
    fi
    
    sleep 1
done

cleanup