#!/bin/bash
# rt_cnc 构建脚本
# 用法: ./build.sh [debug|release|all|clean|help]

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
LIB_DIR="$SCRIPT_DIR/lib"
BUILD_DIR="$SCRIPT_DIR/build"
BUILD_DIR_DEBUG="$BUILD_DIR/debug"
BUILD_DIR_RELEASE="$BUILD_DIR/release"
BIN_DIR="$SCRIPT_DIR/bin"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_help() {
    echo ""
    echo "rt_cnc 构建脚本"
    echo "==============="
    echo ""
    echo "用法: ./build.sh [选项]"
    echo ""
    echo "选项:"
    echo "  debug     编译调试模式 (生成 rt_cnc_d，无需硬件)"
    echo "  release   编译实时模式 (生成 rt_cnc，需要EtherCAT硬件)"
    echo "  all       同时编译调试和实时模式 (默认)"
    echo "  clean     清理所有构建文件"
    echo "  help      显示此帮助信息"
    echo ""
    echo "构建目录结构:"
    echo "  build/"
    echo "    ├── debug/    调试模式构建文件"
    echo "    └── release/  实时模式构建文件"
    echo "  bin/"
    echo "    ├── rt_cnc_d  调试版本可执行文件"
    echo "    └── rt_cnc    实时版本可执行文件"
    echo ""
    echo "示例:"
    echo "  ./build.sh          # 编译所有版本"
    echo "  ./build.sh debug    # 仅编译调试版本"
    echo "  ./build.sh release  # 仅编译发布版本"
    echo ""
}

build_shm_lib() {
    mkdir -p "$LIB_DIR/cncshm/build"
    cd "$LIB_DIR/cncshm/build"
    qmake "../shm.pro"
    make -j$(nproc)
    if [ $? -eq 0 ]; then
        make install
        echo -e "${GREEN}[OK] 编译共享内存库成功, 安装到/usr/local/lib${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 编译共享内存库失败${NC}"
        return 1
    fi
}

build_log_lib() {
    mkdir -p "$LIB_DIR/log/build"
    cd "$LIB_DIR/log/build"
    qmake "../cnclogapp.pro"
    make -j$(nproc)
    if [ $? -eq 0 ]; then
        make install
        echo -e "${GREEN}[OK] 编译日志库成功, 安装到/usr/local/lib${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 编译日志库失败${NC}"
        return 1
    fi
}

build_debug() {
    echo -e "${BLUE}========== 编译调试模式 ==========${NC}"

    mkdir -p "$BUILD_DIR_DEBUG"
    cd "$BUILD_DIR_DEBUG"

    qmake "$SCRIPT_DIR/RT_CNC.pro" "CONFIG+=debug_mode"
    make -j$(nproc)

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 调试模式编译成功${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 调试模式编译失败${NC}"
        return 1
    fi
}

build_release() {
    echo -e "${BLUE}========== 编译实时模式 ==========${NC}"

    mkdir -p "$BUILD_DIR_RELEASE"
    cd "$BUILD_DIR_RELEASE"

    qmake "$SCRIPT_DIR/RT_CNC.pro"
    make -j$(nproc)

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 实时模式编译成功${NC}"
        return 0
    else
        echo -e "${RED}[FAIL] 实时模式编译失败${NC}"
        return 1
    fi
}

build_all() {
    local debug_ok=0
    local release_ok=0

    build_debug && debug_ok=1
    echo ""
    build_release && release_ok=1

    echo ""
    echo -e "${BLUE}========== 编译结果 ==========${NC}"

    if [ $debug_ok -eq 1 ]; then
        echo -e "  调试版本: ${GREEN}[OK]${NC} $BIN_DIR/rt_cnc_d"
    else
        echo -e "  调试版本: ${RED}[FAIL]${NC}"
    fi

    if [ $release_ok -eq 1 ]; then
        echo -e "  实时版本: ${GREEN}[OK]${NC} $BIN_DIR/rt_cnc"
    else
        echo -e "  实时版本: ${RED}[FAIL]${NC}"
    fi

    if [ $debug_ok -eq 1 ] && [ $release_ok -eq 1 ]; then
        echo ""
        echo -e "${GREEN}所有版本编译成功!${NC}"
        return 0
    else
        return 1
    fi
}

clean_build() {
    echo -e "${YELLOW}清理构建文件...${NC}"

    # 清理 共享内存 构建目录
    if [ -d "$LIB_DIR/cncshm/build" ]; then
        rm -rf "$LIB_DIR/cncshm/build"
        echo "  已清理: $LIB_DIR/cncshm/build"
    fi

        # 清理 共享内存 构建目录
    if [ -d "$LIB_DIR/log/build" ]; then
        rm -rf "$LIB_DIR/log/build"
        echo "  已清理: $LIB_DIR/log/build"
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

    # 如果 build 目录为空则删除
    if [ -d "$BUILD_DIR" ] && [ -z "$(ls -A "$BUILD_DIR" 2>/dev/null)" ]; then
        rm -rf "$BUILD_DIR"
        echo "  已清理: $BUILD_DIR"
    fi

    # 清理可执行文件
    if [ -f "$BIN_DIR/rt_cnc" ]; then
        rm -f "$BIN_DIR/rt_cnc"
        echo "  已清理: $BIN_DIR/rt_cnc"
    fi
    if [ -f "$BIN_DIR/rt_cnc_d" ]; then
        rm -f "$BIN_DIR/rt_cnc_d"
        echo "  已清理: $BIN_DIR/rt_cnc_d"
    fi

    echo -e "${GREEN}清理完成!${NC}"
}

# 主逻辑
case "${1:-all}" in
    debug)
        build_shm_lib
        build_log_lib
        build_debug
        echo ""
        echo -e "可执行文件: ${YELLOW}$BIN_DIR/rt_cnc_d${NC}"
        ;;
    release)
        build_shm_lib
        build_log_lib
        build_release
        echo ""
        echo -e "可执行文件: ${YELLOW}$BIN_DIR/rt_cnc${NC}"
        ;;
    all)
        build_shm_lib
        build_log_lib
        build_all
        ;;
    clean)
        clean_build
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
