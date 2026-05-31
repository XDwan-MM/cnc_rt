#!/bin/bash

# ================= 配置区域 =================
# 定义要执行的命令 (保持你的原始命令不变)
CMD2="sudo -E LD_LIBRARY_PATH=./cnc_rt/lib/cncshm/bin ./cnc_rt/bin/rt_cnc_1"

# 终端模拟器选择策略:
# 常见选项: gnome-terminal, konsole, xterm, xfce4-terminal, mate-terminal
# 脚本会自动尝试第一个可用的
TERMINAL_OPTIONS=("gnome-terminal" "konsole" "xfce4-terminal" "mate-terminal" "xterm")
# ============================================

echo "=========================================="
echo "CNC Process Launcher"
echo "=========================================="

# 1. 查找可用的终端模拟器
TERMINAL_CMD=""
for term in "${TERMINAL_OPTIONS[@]}"; do
    if command -v "$term" &> /dev/null; then
        TERMINAL_CMD="$term"
        break
    fi
done

if [ -z "$TERMINAL_CMD" ]; then
    echo "[ERROR] No supported terminal emulator found."
    echo "Please install one of: gnome-terminal, konsole, xterm, etc."
    exit 1
fi

echo "[INFO] Using terminal emulator: $TERMINAL_CMD"
echo "[INFO] Starting processes in new windows..."
echo "[INFO] Press Ctrl+C in THIS window to stop all."
echo "------------------------------------------"

# 存储子进程 PID 的数组
CHILD_PIDS=()

# 2. 定义启动函数
launch_process() {
    local cmd="$1"
    local name="$2"
    
    # 根据终端类型构建启动命令
    # 大多数现代终端 (gnome/konsole/xfce) 使用 -- bash -c "cmd; exec bash" 来保持窗口打开
    # xterm 使用 -e "cmd"
    
    local launch_cmd=""
    case "$TERMINAL_CMD" in
        xterm)
            launch_cmd="$TERMINAL_CMD -e \"$cmd\""
            ;;
        *)
            # 对于 gnome-terminal, konsole 等，使用 bash -c 包装以保留环境变量和 sudo 上下文
            # exec bash 确保命令结束后窗口不立即关闭，方便调试（虽然我们会 kill 它）
            launch_cmd="$TERMINAL_CMD -- bash -c \"$cmd; exec bash\""
            ;;
    esac
    
    # 后台执行新终端
    eval "$launch_cmd" &
    
    # 获取新终端窗口的 PID
    local pid=$!
    CHILD_PIDS+=($pid)
    
    echo "[STARTED] $name (PID: $pid)"
}

# 3. 启动两个进程
launch_process "$CMD1" "RT_CNC_X"

# 检查是否成功启动
if [ ${#CHILD_PIDS[@]} -eq 0 ]; then
    echo "[ERROR] Failed to start any processes."
    exit 1
fi

# 4. 定义清理函数 (处理 Ctrl+C)
cleanup() {
    echo ""
    echo "--------------------------------------------------"
    echo "Received SIGINT (Ctrl+C). Shutting down processes..."
    
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Sending SIGTERM to PID $pid..."
            kill -TERM "$pid" 2>/dev/null
        else
            echo "PID $pid is already dead."
        fi
    done
    
    # 等待进程退出
    sleep 2
    
    # 强制杀死残留进程
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Force killing PID $pid..."
            kill -9 "$pid" 2>/dev/null
        fi
    done
    
    echo "All processes terminated."
    echo "--------------------------------------------------"
    exit 0
}

# 5. 绑定信号处理
trap cleanup SIGINT SIGTERM

# 6. 主循环：保持脚本运行，直到用户按下 Ctrl+C 或所有子进程自然退出
while true; do
    # all_dead=true
    # for pid in "${CHILD_PIDS[@]}"; do
    #     if kill -0 "$pid" 2>/dev/null; then
    #         all_dead=false
    #         break
    #     fi
    # done
    
    # if [ "$all_dead" = true ]; then
    #     echo "[INFO] All child processes exited naturally."
    #     break
    # fi
    
    sleep 1
done

# wait ${CHILD_PIDS[@]}

# 如果是因为子进程自然退出而结束，也执行一次清理（虽然可能已经死了）
cleanup
