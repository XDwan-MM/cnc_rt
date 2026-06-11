#!/bin/bash

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
EXISTING_PIDS=$(ps -eo pid,comm | awk -v mypid="$$" '$1 != mypid && ($2 ~ /^rt_cnc/ || $2 == "cnc_hmi_v3") {print $1}')

if [ -n "$EXISTING_PIDS" ]; then
    # 将多个 PID 转为单行空格分隔，方便显示
    DISPLAY_PIDS=$(echo $EXISTING_PIDS | tr '\n' ' ')
    echo -e "${COLOR_ERR}[警告] 发现已有相关进程正在运行 (PID: $DISPLAY_PIDS)${COLOR_RESET}"
    read -p "是否强制终止这些进程并继续运行？ [y/N]: " choice </dev/tty
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
HMI_BIN_DIR="${HMI_DIRS[0]%/}/bin"
RT_BIN_DIR="${RT_DIRS[0]%/}/bin"

# ==================== 颜色定义 ====================
COLOR_HMI=$'\e[1;32m'    # 绿色 (加粗)
COLOR_RT=$'\e[1;33m'     # 黄色 (加粗)
COLOR_SYS=$'\e[1;36m'    # 青色 (加粗)
COLOR_ERR=$'\e[1;31m'    # 红色 (加粗)
COLOR_RESET=$'\e[0m'     # 重置颜色

# ==================== 运行与监控部分 ====================

PID1=""
PID2=""

# 定义清理函数：用于关闭所有后台进程
cleanup() {
    # 移除 trap 避免死循环
    trap - EXIT SIGINT SIGTERM
    
    echo -e "\n${COLOR_SYS}[系统] 正在关闭所有相关后台进程...${COLOR_RESET}"
    if [ -n "$PID1" ]; then
        echo "${COLOR_SYS}[系统] 正在停止界面进程 (PID: $PID1)...${COLOR_RESET}"
        sudo kill "$PID1" 2>/dev/null
    fi
    if [ -n "$PID2" ]; then
        echo "${COLOR_SYS}[系统] 正在停止实时进程 (PID: $PID2)...${COLOR_RESET}"
        sudo kill "$PID2" 2>/dev/null
    fi
    exit 0
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
    echo "  run_d      运行调试仿真模式"
    echo "  run_1      运行平台1"
    echo "  run_2      运行平台2"
    echo "  help       显示此帮助信息"
    echo ""
}

RT_PAR=""
HMI_PAR=""

# 主逻辑
case "${1:-all}" in
    1|run1|run_1)
        RT_PAR=cnc_rt_1
        ;;
    2|run2|run_2)
        RT_PAR=cnc_rt_2
        ;;
    all|run|rund|run_d|run_debug|debug_run)
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

# 定义清理函数：用于关闭所有后台进程并清理环境（退出时保障）
cleanup() {
    # 移除 trap 避免死循环
    trap - EXIT SIGINT SIGTERM
    
    # \r 保证不管终端当前状态如何，都能回到行首打印
    echo -e "\r\n${COLOR_SYS}[系统] 正在关闭所有相关后台进程并清理环境...${COLOR_RESET}"
    
    # 1. 优先杀掉已知 PID 
    if [ -n "$PID1" ]; then
        echo -e "${COLOR_SYS}[系统] 正在停止界面进程 (PID: $PID1)...${COLOR_RESET}\r"
        sudo kill -9 "$PID1" 2>/dev/null
    fi
    if [ -n "$PID2" ]; then
        echo -e "${COLOR_SYS}[系统] 正在停止实时进程 (PID: $PID2)...${COLOR_RESET}\r"
        sudo kill -9 "$PID2" 2>/dev/null
    fi
    
    # 2. 兜底扫描，保证【绝对】没有存活的同名进程
    local LEFT_PIDS=$(ps -eo pid,comm | awk -v mypid="$$" '$1 != mypid && ($2 ~ /^cnc_rt/ || $2 == "cnc_hmi_v3") {print $1}')
    if [ -n "$LEFT_PIDS" ]; then
        sudo kill -9 $LEFT_PIDS 2>/dev/null
    fi
    
    # 3. 等待 0.5 秒，确保进程死亡后，操作系统将其与共享内存的连接数清零
    sleep 0.5
    
    # 4. 彻底扫除任何大小超过 10000000 字节且无连接的共享内存段
    clean_shm
    
    # 恢复终端正常状态（防止因为进程修改 TTY 导致终端回车变阶梯）
    stty sane 2>/dev/null
    echo -e "${COLOR_SYS}[系统] 脚本已安全退出，环境清理完毕。${COLOR_RESET}"
    exit 0
}


echo " "
echo "${COLOR_SYS}=========================================="
echo "${COLOR_SYS}                格力数控系统                "
echo "${COLOR_SYS}=========================================="
echo " "
echo "${COLOR_SYS}[TIPS] 按 Ctrl+C 可以关闭系统。              "
echo " "


# 捕获 Ctrl+C (SIGINT)、终止信号 (SIGTERM) 以及脚本正常退出事件 (EXIT)
trap cleanup EXIT SIGINT SIGTERM

# 4. 启动界面进程（后台运行，使用 awk 强制行缓冲并添加 \r 修复错位）
cd "$HMI_BIN_DIR"
sudo -E LD_LIBRARY_PATH="${HMI_DIRS[0]}/include/cncshm/bin/" stdbuf -oL -eL "${HMI_DIRS[0]}/bin/cnc_hmi_v3" ${HMI_PAR} 2>&1 | \
    awk -v prefix="${COLOR_HMI}[HMI]${COLOR_RESET}" '{sub(/\r$/, ""); print prefix " " $0 "\r"; fflush()}' &
PID1=$! 
echo "${COLOR_SYS}[系统] 界面已在后台启动，PID: $PID1${COLOR_RESET}"

# 5. 延时 1 秒
echo "${COLOR_SYS}[系统] 等待 1 秒...${COLOR_RESET}"
sleep 1

# 6. 启动实时进程（后台运行，使用 awk 强制行缓冲并添加 \r 修复错位）
cd "$RT_BIN_DIR"
sudo -E LD_LIBRARY_PATH="${RT_DIRS[0]}/lib/cncshm/bin/" stdbuf -oL -eL "${RT_DIRS[0]}/bin/${RT_PAR}" 2>&1 | \
    awk -v prefix="${COLOR_RT}[RT]${COLOR_RESET}" '{sub(/\r$/, ""); print prefix " " $0 "\r"; fflush()}' &
PID2=$! 
echo "${COLOR_SYS}[系统] 实时进程已在后台启动，PID: $PID2${COLOR_RESET}"

echo "${COLOR_SYS}[系统] 两个进程均已启动，开始监控运行状态。${COLOR_RESET}"

# 7. 循环监控：若任何一个进程意外退出，则关闭另一个并退出脚本
while true; do
    # 检查进程1是否存在
    if ! sudo kill -0 "$PID1" 2>/dev/null; then
        echo -e "\r\n${COLOR_SYS}[系统] 检测到 [CNC] 界面进程已意外退出。${COLOR_RESET}"
        cleanup
    fi
    
    # 检查进程2是否存在
    if ! sudo kill -0 "$PID2" 2>/dev/null; then
        echo -e "\r\n${COLOR_SYS}[系统] 检测到 [RT] 实时进程已意外退出。${COLOR_RESET}"
        cleanup
    fi
    
    sleep 1
done
