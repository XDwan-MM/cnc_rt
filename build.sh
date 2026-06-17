#!/bin/bash
# cnc_rt 构建脚本
# 用法: ./build.sh [debug|release|all|debug_run|run|run2|clean|help]

set -e

# export PATH=$PATH:/opt/Qt5.15.16/5.15.16/gcc_64/bin/

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
LIB_DIR="$SCRIPT_DIR/lib"
BUILD_DIR="$SCRIPT_DIR/build"
BUILD_DIR_DEBUG="$BUILD_DIR/debug"
BUILD_DIR_RELEASE="$BUILD_DIR/release"
BUILD_DIR_RELEASE1="$BUILD_DIR/release1"
BUILD_DIR_RELEASE2="$BUILD_DIR/release2"
BIN_DIR="$SCRIPT_DIR/bin"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_help() {
    echo ""
    echo "cnc_rt 构建脚本"
    echo "==============="
    echo ""
    echo "用法: ./build.sh [选项]"
    echo ""
    echo "选项:"
    echo "  debug      编译调试模式 (生成 cnc_rt_d，无需硬件)"
    echo "  release    编译实时模式 (生成 cnc_rt，需要EtherCAT硬件)"
    echo "  all        同时编译调试和实时模式 (默认)"
    echo "  debug_run  编译调试模式并自动运行 cnc_rt_d"
    echo "  run        编译实时模式平台1并运行 cnc_rt_1"
    echo "  run2       编译实时模式平台2并运行 cnc_rt_2"
    echo "  clean      清理所有构建文件"
    echo "  help       显示此帮助信息"
    echo ""
    echo "构建目录结构:"
    echo "  build/"
    echo "    ├── debug/    调试模式构建文件"
    echo "    └── release/  实时模式构建文件"
    echo "  bin/"
    echo "    ├── cnc_rt_d  调试版本可执行文件"
    echo "    └── cnc_rt    实时版本可执行文件"
    echo ""
    echo "示例:"
    echo "  ./build.sh          # 编译所有版本"
    echo "  ./build.sh debug    # 仅编译调试版本"
    echo "  ./build.sh release    # 仅编译发布版本"
    echo "  ./build.sh debug_run  # 编译调试模式并运行"
    echo ""
}

build_shm_lib() {
    mkdir -p "$LIB_DIR/cncshm/build"
    cd "$LIB_DIR/cncshm/build"
    qmake ../shm.pro CONFIG+=debug
    make -j$(nproc)
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 编译共享内存库成功"
        return 0
    else
        echo -e "${RED}[FAIL] 编译共享内存库失败"
        return 1
    fi
}

build_debug() {
    echo -e "${BLUE}========== 编译调试模式 ==========${NC}"

    mkdir -p "$BUILD_DIR_DEBUG"
    cd "$BUILD_DIR_DEBUG"

    qmake "$SCRIPT_DIR/CNC_RT.pro" "CONFIG+=debug_mode"
    make -j$(nproc)

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 调试模式编译成功${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 调试模式编译失败${NC}"
        return 1
    fi
}

build_release_1() {
    echo -e "${BLUE}========== 编译实时模式 ==========${NC}"

    mkdir -p "$BUILD_DIR_RELEASE1"
    cd "$BUILD_DIR_RELEASE1"

    qmake "$SCRIPT_DIR/CNC_RT.pro" "CONFIG+=release_platform_1"
    make -j$(nproc)

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 实时模式编译平台1成功${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 实时模式编译平台1失败${NC}"
        return 1
    fi
}

build_release_2() {
    echo -e "${BLUE}========== 编译实时模式 ==========${NC}"

    mkdir -p "$BUILD_DIR_RELEASE2"
    cd "$BUILD_DIR_RELEASE2"

    qmake "$SCRIPT_DIR/CNC_RT.pro" "CONFIG+=release_platform_2"
    make -j$(nproc)

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 实时模式编译平台2成功${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 实时模式编译平台2失败${NC}"
        return 1
    fi
}

build_all() {
    local debug_ok=0
    local release1_ok=0
    local release2_ok=0

    build_debug && debug_ok=1
    echo ""
    build_release_1 && release1_ok=1
    echo ""
    build_release_2 && release2_ok=1

    echo ""
    echo -e "${BLUE}========== 编译结果 ==========${NC}"

    if [ $debug_ok -eq 1 ]; then
        echo -e "  调试版本: ${GREEN}[OK]${NC} $BIN_DIR/cnc_rt_d"
    else
        echo -e "  调试版本: ${RED}[FAIL]${NC}"
    fi

    if [ $release1_ok -eq 1 ]; then
        echo -e "  实时版本平台1: ${GREEN}[OK]${NC} $BIN_DIR/cnc_rt_1"
    else
        echo -e "  实时版本平台1: ${RED}[FAIL]${NC}"
    fi

    if [ $release2_ok -eq 1 ]; then
        echo -e "  实时版本平台2: ${GREEN}[OK]${NC} $BIN_DIR/cnc_rt_2"
    else
        echo -e "  实时版本平台2: ${RED}[FAIL]${NC}"
    fi

    if [ $debug_ok -eq 1 ] && [ $release1_ok -eq 1 ] && [ $release2_ok -eq 1 ]; then
        echo ""
        echo -e "${GREEN}所有版本编译成功!${NC}"
        return 0
    else
        return 1
    fi
}

run() {
    build_shm_lib
    build_release_1
    cd $BIN_DIR
    sudo -E ./cnc_rt_1
}

run2() {
    build_shm_lib
    build_release_2
    cd $BIN_DIR
    sudo -E ./cnc_rt_2
}

run_debug() {
    build_shm_lib
    build_debug
    cd $BIN_DIR
    sudo -E ./cnc_rt_d
}

clean_build() {
    echo -e "${YELLOW}清理构建文件...${NC}"

    # 清理 共享内存 构建目录
    if [ -d "$LIB_DIR/cncshm/build" ]; then
        rm -rf "$LIB_DIR/cncshm/build"
        echo "  已清理: $LIB_DIR/cncshm/build"
    fi

    if [ -d "$LIB_DIR/cncshm/bin" ]; then
        rm -rf "$LIB_DIR/cncshm/bin"
        echo "  已清理: $LIB_DIR/cncshm/bin"
    fi

    # 清理 debug 构建目录
    if [ -d "$BUILD_DIR_DEBUG" ]; then
        rm -rf "$BUILD_DIR_DEBUG"
        echo "  已清理: $BUILD_DIR_DEBUG"
    fi

    # 清理 release 构建目录
    if [ -d "$BUILD_DIR_RELEASE" ]; then
        rm -rf "$BUILD_DIR_RELEASE"
        echo "  已清理: $BUILD_DIR_RELEASE"
    fi
    if [ -d "$BUILD_DIR_RELEASE1" ]; then
        rm -rf "$BUILD_DIR_RELEASE1"
        echo "  已清理: $BUILD_DIR_RELEASE1"
    fi
    if [ -d "$BUILD_DIR_RELEASE2" ]; then
        rm -rf "$BUILD_DIR_RELEASE2"
        echo "  已清理: $BUILD_DIR_RELEASE2"
    fi

    # 如果 build 目录为空则删除
    if [ -d "$BUILD_DIR" ] && [ -z "$(ls -A "$BUILD_DIR" 2>/dev/null)" ]; then
        rm -rf "$BUILD_DIR"
        echo "  已清理: $BUILD_DIR"
    fi

    # 清理可执行文件
    if [ -f "$BIN_DIR/cnc_rt" ]; then
        rm -f "$BIN_DIR/cnc_rt"
        echo "  已清理: $BIN_DIR/cnc_rt"
    fi
    if [ -f "$BIN_DIR/cnc_rt_1" ]; then
        rm -f "$BIN_DIR/cnc_rt_1"
        echo "  已清理: $BIN_DIR/cnc_rt_1"
    fi
    if [ -f "$BIN_DIR/cnc_rt_2" ]; then
        rm -f "$BIN_DIR/cnc_rt_2"
        echo "  已清理: $BIN_DIR/cnc_rt_2"
    fi
    if [ -f "$BIN_DIR/cnc_rt_d" ]; then
        rm -f "$BIN_DIR/cnc_rt_d"
        echo "  已清理: $BIN_DIR/cnc_rt_d"
    fi

    echo -e "${GREEN}清理完成!${NC}"
}

# 主逻辑
case "${1:-all}" in
    debug)
        build_shm_lib
        build_debug
        echo ""
        echo -e "可执行文件: ${YELLOW}$BIN_DIR/cnc_rt_d${NC}"
        ;;
    release)
        build_shm_lib
        build_release_1
        build_release_2
        echo ""
        echo -e "可执行文件: ${YELLOW}$BIN_DIR/cnc_rt${NC}"
        ;;
    check)
        build_release_1
        echo ""
        echo -e "可执行文件: ${YELLOW}$BIN_DIR/cnc_rt${NC}"
        ;;
    all)
        build_shm_lib
        build_all
        ;;
    clean)
        clean_build
        ;;
    run|run1|run_1)
        run
        ;;
    run2|run_2)
        run2
        ;;
    rund|run_d|run_debug|debug_run)
        run_debug
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        echo -e "${RED}未知选项: $1${NC}"
        print_help
        exit 1
        ;;
esac
