#!/bin/bash

# ==================== 颜色定义 ====================
COLOR_HMI=$'\e[1;32m'    # 绿色 (加粗)
COLOR_RT=$'\e[1;33m'     # 黄色 (加粗)
COLOR_SYS=$'\e[1;36m'    # 青色 (加粗)
COLOR_ERR=$'\e[1;31m'    # 红色 (加粗)
COLOR_RESET=$'\e[0m'     # 重置颜色
RED=$'\e[1;31m'
NC=$'\e[0m'

# 获取当前脚本所在目录
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# 开启 nullglob：若无匹配则返回空数组，而非保留字面量通配符
shopt -s nullglob

# 匹配名称中包含 hmi 或 rt 的目录（末尾加 / 确保仅匹配目录）
HMI_DIRS=("$SCRIPT_DIR"/*hmi*/)
RT_DIRS=("$SCRIPT_DIR"/*rt*/)

# 关闭 nullglob
shopt -u nullglob

# 严格校验：必须恰好匹配 1 个目录，否则报错并退出
if [ ${#HMI_DIRS[@]} -ne 1 ]; then
    echo "${COLOR_ERR}[错误] 期望恰好匹配 1 个包含 'hmi' 的目录，实际匹配到 ${#HMI_DIRS[@]} 个${COLOR_RESET}" >&2
    exit 1
fi
if [ ${#RT_DIRS[@]} -ne 1 ]; then
    echo "${COLOR_ERR}[错误] 期望恰好匹配 1 个包含 'rt' 的目录，实际匹配到 ${#RT_DIRS[@]} 个${COLOR_RESET}" >&2
    exit 1
fi

# ==================== 新增：清理共享内存函数 ====================
clean_shm() {
    # 解析 ipcs -m，寻找：第2列是数字(shmid)、第5列(字节)>=10000000、第6列(连接数)==0 的段
    local shmids=$(ipcs -m | awk '$2 ~ /^[0-9]+$/ && $5 >= 10000000 && $6 == 0 {print $2}')
    if [ -n "$shmids" ]; then
        for id in $shmids; do
            echo -e "${COLOR_SYS}[系统] 发现无连接的大块共享内存段 (shmid: $id)，正在执行清理...${COLOR_RESET}\r"
            sudo ipcrm -m "$id" 2>/dev/null
        done
    fi
}

# ==================== 新增：启动前环境检查 ====================
echo -e "${COLOR_SYS}[系统] 正在检查运行环境...${COLOR_RESET}"

# 寻找残留进程（排除当前脚本进程自身）
EXISTING_PIDS=$(ps -eo pid,comm | awk -v mypid="$$" '$1 != mypid && ($2 ~ /^cnc_rt/ || $2 == "cnc_hmi_v3") {print $1}')

if [ -n "$EXISTING_PIDS" ]; then
    # 将多个 PID 转为单行空格分隔，方便显示
    DISPLAY_PIDS=$(echo $EXISTING_PIDS | tr '\n' ' ')
    echo -e "${COLOR_ERR}[警告] 发现已有相关进程正在运行 (PID: $DISPLAY_PIDS)${COLOR_RESET}"
    read -p "是否强制终止这些进程并继续运行？ [Y/N]: " choice </dev/tty
    case "$choice" in
        y|Y )
            echo -e "${COLOR_SYS}[系统] 正在清理残留进程...${COLOR_RESET}"
            sudo kill -9 $EXISTING_PIDS 2>/dev/null
            sleep 0.5 # 给予系统 0.5 秒时间完全回收进程，并断开对共享内存的占用
            ;;
        * )
            echo -e "${COLOR_SYS}[系统] 用户取消，脚本已退出。${COLOR_RESET}"
            exit 0
            ;;
    esac
fi

# 检查并清理遗留的大块空闲共享内存（启动前确保干净）
clean_shm

# 去除路径末尾的 '/'，安全拼接 bin 目录
HMI_ROOT="${HMI_DIRS[0]%/}"
RT_ROOT="${RT_DIRS[0]%/}"
HMI_BIN_DIR="${HMI_ROOT}/bin"
RT_BIN_DIR="${RT_ROOT}/bin"

# ==================== 运行与监控部分 ====================

HMI_PID=""
RT_PID=""
HMI_PIPE_PID=""
RT_PIPE_PID=""
PID_DIR=""
HMI_PID_FILE=""
RT_PID_FILE=""
CLEANUP_RUNNING=0

process_alive() {
    local pid="$1"
    [ -n "$pid" ] && ps -p "$pid" -o stat= 2>/dev/null | awk '$1 !~ /^Z/ {found=1} END {exit !found}'
}

wait_for_process_exit() {
    local pid="$1"
    local attempts="$2"
    local index
    for ((index = 0; index < attempts; ++index)); do
        if ! process_alive "$pid"; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

wait_for_pid_file() {
    local pid_file="$1"
    local process_name="$2"
    local index
    local pid
    for ((index = 0; index < 50; ++index)); do
        if [ -s "$pid_file" ]; then
            pid=$(<"$pid_file")
            if [[ "$pid" =~ ^[0-9]+$ ]] && process_alive "$pid"; then
                return 0
            fi
        fi
        sleep 0.1
    done

    echo -e "${COLOR_ERR}[错误] 无法获取 ${process_name} 的实际进程 PID。${COLOR_RESET}" >&2
    return 1
}

stop_hmi() {
    if ! process_alive "$HMI_PID"; then
        return
    fi

    echo -e "${COLOR_SYS}[系统] 正在通知界面保存保持寄存器并退出...${COLOR_RESET}\r"
    sudo kill -INT "$HMI_PID" 2>/dev/null
    if wait_for_process_exit "$HMI_PID" 50; then
        return
    fi

    echo -e "${COLOR_ERR}[警告] 界面进程 5 秒内未退出，发送 SIGTERM。${COLOR_RESET}\r"
    sudo kill -TERM "$HMI_PID" 2>/dev/null
    if wait_for_process_exit "$HMI_PID" 20; then
        return
    fi

    echo -e "${COLOR_ERR}[警告] 界面进程仍未退出，执行强制终止。${COLOR_RESET}\r"
    sudo kill -KILL "$HMI_PID" 2>/dev/null
    wait_for_process_exit "$HMI_PID" 10
}

stop_rt() {
    if ! process_alive "$RT_PID"; then
        return
    fi

    echo -e "${COLOR_SYS}[系统] 正在停止实时进程...${COLOR_RESET}\r"
    sudo kill -TERM "$RT_PID" 2>/dev/null
    if wait_for_process_exit "$RT_PID" 30; then
        return
    fi

    echo -e "${COLOR_ERR}[警告] 实时进程 3 秒内未退出，执行强制终止。${COLOR_RESET}\r"
    sudo kill -KILL "$RT_PID" 2>/dev/null
    wait_for_process_exit "$RT_PID" 10
}

stop_output_pipe() {
    local pipe_pid="$1"
    if process_alive "$pipe_pid"; then
        kill -TERM "$pipe_pid" 2>/dev/null
    fi
    if [ -n "$pipe_pid" ]; then
        wait "$pipe_pid" 2>/dev/null
    fi
}

# 定义清理函数：用于关闭所有后台进程并清理环境（退出时保障）
cleanup() {
    local exit_code="${1:-0}"
    if [ "$CLEANUP_RUNNING" -eq 1 ]; then
        return
    fi
    CLEANUP_RUNNING=1

    # 清理期间忽略后续退出信号，避免中断寄存器保存流程
    trap - EXIT
    trap '' SIGINT SIGTERM
    
    # \r 保证不管终端当前状态如何，都能回到行首打印
    echo -e "\r\n${COLOR_SYS}[系统] 正在关闭所有相关后台进程并清理环境...${COLOR_RESET}"
    
    # 1. 先让界面保存保持寄存器，再停止实时进程
    stop_hmi
    stop_rt

    # 2. 业务进程退出后再关闭日志输出管道
    stop_output_pipe "$HMI_PIPE_PID"
    stop_output_pipe "$RT_PIPE_PID"
    
    # 3. 最终兜底：只强制终止优雅退出超时后仍存活的同名进程
    local LEFT_PIDS=$(ps -eo pid,comm | awk -v mypid="$$" '$1 != mypid && ($2 ~ /^cnc_rt/ || $2 == "cnc_hmi_v3") {print $1}')
    if [ -n "$LEFT_PIDS" ]; then
        echo -e "${COLOR_ERR}[警告] 发现退出超时的残留进程，执行最终清理：${LEFT_PIDS//$'\n'/ }${COLOR_RESET}\r"
        sudo kill -KILL $LEFT_PIDS 2>/dev/null
    fi
    
    # 4. 等待进程死亡后，操作系统将其与共享内存的连接数清零
    sleep 0.5
    
    # 5. 彻底扫除任何大小超过 10000000 字节且无连接的共享内存段
    clean_shm

    if [ -n "$HMI_PID_FILE" ]; then
        rm -f "$HMI_PID_FILE"
    fi
    if [ -n "$RT_PID_FILE" ]; then
        rm -f "$RT_PID_FILE"
    fi
    if [ -n "$PID_DIR" ]; then
        rmdir "$PID_DIR" 2>/dev/null
    fi
    
    # 恢复终端正常状态（防止因为进程修改 TTY 导致终端回车变阶梯）
    stty sane 2>/dev/null
    echo -e "${COLOR_SYS}[系统] 脚本已安全退出，环境清理完毕。${COLOR_RESET}"
    exit "$exit_code"
}

print_help() {
    echo ""
    echo "数控系统启动脚本帮助"
    echo "====================="
    echo ""
    echo "用法: ./run.sh [选项]"
    echo ""
    echo "选项:"
    echo "             运行调试仿真模式（默认）"
    echo "  debug      运行调试仿真模式"
    echo "  run        运行实时平台"
    echo "  help       显示此帮助信息"
    echo ""
}

RT_PAR=""
HMI_PAR=""

# 主逻辑
case "${1:-all}" in
    run)
        RT_PAR=cnc_rt
        ;;
    all|debug|rund|run_d|run_debug|debug_run)
        RT_PAR=cnc_rt_d
        HMI_PAR=-p
        ;;
    help|--help|-h)
        print_help
        exit 1
        ;;
    *)
        echo -e "${RED}未知选项: $1${NC}"
        print_help
        exit 1
        ;;
esac


echo " "
echo "${COLOR_SYS}=========================================="
echo "${COLOR_SYS}                格力数控系统                "
echo "${COLOR_SYS}=========================================="
echo " "
echo "${COLOR_SYS}[TIPS] 按 Ctrl+C 可以关闭系统。              "
echo " "

# 捕获 Ctrl+C (SIGINT)、终止信号 (SIGTERM) 以及脚本正常退出事件 (EXIT)
trap 'cleanup 130' SIGINT
trap 'cleanup 143' SIGTERM
trap 'exit_code=$?; cleanup "$exit_code"' EXIT

PID_DIR=$(mktemp -d "${TMPDIR:-/tmp}/cnc_launcher.XXXXXX")
if [ -z "$PID_DIR" ]; then
    echo -e "${COLOR_ERR}[错误] 无法创建进程 PID 临时目录。${COLOR_RESET}" >&2
    exit 1
fi
HMI_PID_FILE="${PID_DIR}/hmi.pid"
RT_PID_FILE="${PID_DIR}/rt.pid"

# 4. 启动界面进程（两个进程统一使用 sudo，避免一个 root 一个普通用户）
cd "$HMI_BIN_DIR"
sudo bash -c 'pid_file=$1; shift; echo $$ > "$pid_file"; exec stdbuf -oL -eL "$@"' \
    bash "$HMI_PID_FILE" "${HMI_ROOT}/bin/cnc_hmi_v3" ${HMI_PAR} 2>&1 | \
    while IFS= read -r line; do
        printf "%s %s\r\n" "${COLOR_HMI}[HMI]${COLOR_RESET}" "${line%$'\r'}"
    done &
HMI_PIPE_PID=$!
if ! wait_for_pid_file "$HMI_PID_FILE" "界面进程"; then
    cleanup 1
fi
HMI_PID=$(<"$HMI_PID_FILE")
echo "${COLOR_SYS}[系统] 界面已在后台启动，PID: ${HMI_PID}。${COLOR_RESET}"

# 5. 延时 1 秒
echo "${COLOR_SYS}[系统] 等待 1 秒...${COLOR_RESET}"
sleep 1

# 6. 启动实时进程（两个进程统一使用 sudo，避免一个 root 一个普通用户）
cd "$RT_BIN_DIR"
sudo bash -c 'pid_file=$1; shift; echo $$ > "$pid_file"; exec stdbuf -oL -eL "$@"' \
    bash "$RT_PID_FILE" "${RT_ROOT}/bin/${RT_PAR}" 2>&1 | \
    while IFS= read -r line; do
        printf "%s %s\r\n" "${COLOR_RT}[RT]${COLOR_RESET}" "${line%$'\r'}"
    done &
RT_PIPE_PID=$!
if ! wait_for_pid_file "$RT_PID_FILE" "实时进程"; then
    cleanup 1
fi
RT_PID=$(<"$RT_PID_FILE")
echo "${COLOR_SYS}[系统] 实时进程已在后台启动，PID: ${RT_PID}。${COLOR_RESET}"

echo "${COLOR_SYS}[系统] 两个进程均已启动，开始监控运行状态。按 Ctrl+C 可以同时停止它们。${COLOR_RESET}"

# 7. 循环监控：若任何一个进程意外退出，则关闭另一个并退出脚本
while true; do
    if ! process_alive "$HMI_PID"; then
        echo -e "\r\n${COLOR_SYS}[系统] 检测到 [HMI] 界面进程已意外退出。${COLOR_RESET}"
        cleanup 1
    fi
    
    if ! process_alive "$RT_PID"; then
        echo -e "\r\n${COLOR_SYS}[系统] 检测到 [RT] 实时进程已意外退出。${COLOR_RESET}"
        cleanup 1
    fi
    
    sleep 1
done
