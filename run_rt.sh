#!/bin/bash

# 设置错误退出模式
set -e

# 定义要运行的命令和参数
CMD2="sudo -E LD_LIBRARY_PATH=./cnc_rt/lib/cncshm/bin ./cnc_rt/bin/rt_cnc_1"

echo "Starting CNC processes..."
echo "Press Ctrl+C to stop both processes."

# 启动第二个进程，并获取其 PID
$CMD2 &
PID2=$!

# 将这两个 PID 存储在一个数组中，方便后续清理
PIDS=($PID2)

# 定义清理函数
cleanup() {
    echo ""
    echo "Received shutdown signal. Stopping processes..."
    
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Sending SIGTERM to process $pid..."
            kill -TERM "$pid" 2>/dev/null
        fi
    done
    
    # 等待一小段时间让进程优雅退出
    sleep 1
    
    # 如果进程还在，强制杀死
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Force killing process $pid..."
            kill -9 "$pid" 2>/dev/null
        fi
    done
    
    echo "All processes stopped."
    exit 0
}

# 捕获 SIGINT (Ctrl+C) 和 SIGTERM (kill 命令)
trap cleanup SIGINT SIGTERM

# 等待任何一个子进程结束（这样脚本不会立即退出）
wait ${PIDS[@]}

# 如果是因为某个进程自然退出而到达这里，也执行清理
cleanup
