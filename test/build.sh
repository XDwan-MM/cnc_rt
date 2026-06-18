#!/bin/bash
set -e

# 获取脚本所在目录（确保在 test 目录执行）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
TMP_BUILD_DIR="${TMPDIR:-/tmp}/cnc_rt_test_build_${USER:-$(id -u)}"
SHM_BUILD_DIR="$SCRIPT_DIR/../lib/cncshm/build"
SHM_PRO="$SCRIPT_DIR/../lib/cncshm/shm.pro"
cd "$SCRIPT_DIR"

echo "当前目录: $(pwd)"

clean_build(){
    # 强制删除 build（包括处理权限问题）
    if [ -d "$BUILD_DIR" ]; then
        echo "发现 build 目录，正在删除..."
        rm -rf "$BUILD_DIR" 2>/dev/null || sudo rm -rf "$BUILD_DIR"
    fi

    if [ -d "$TMP_BUILD_DIR" ]; then
        echo "发现临时 build 目录，正在删除..."
        rm -rf "$TMP_BUILD_DIR" 2>/dev/null || sudo rm -rf "$TMP_BUILD_DIR"
    fi

    # 删除测试 bin（生成的可执行文件目录）
    if [ -d "bin" ]; then
        echo "发现 bin 目录，正在删除..."
        rm -rf bin 2>/dev/null || sudo rm -rf bin
    fi

    echo "已删除"
}

build_shm_lib(){
    echo "构建共享内存静态库 libshm.a..."
    mkdir -p "$SHM_BUILD_DIR"
    cd "$SHM_BUILD_DIR"

    echo "执行 qmake..."
    qmake "$SHM_PRO" CONFIG+=debug

    echo "执行 make..."
    make -j$(nproc)

    cd "$SCRIPT_DIR"
    echo "libshm.a 构建完成！"
}

build_test(){
    # build_shm_lib

    # 重新创建并构建
    local build_dir="$BUILD_DIR"
    if [ -d "$build_dir" ] && [ ! -w "$build_dir" ]; then
        build_dir="$TMP_BUILD_DIR"
        echo "build 目录不可写，改用临时构建目录: $build_dir"
    fi

    echo "创建/复用 build 目录..."
    mkdir -p "$build_dir"
    cd "$build_dir"

    echo "执行 qmake..."
    qmake "$SCRIPT_DIR"

    echo "执行 make..."
    make -j$(nproc)

    echo "构建完成！"
}

run_test(){
    cd "$SCRIPT_DIR"
    if [ "$(id -u)" -eq 0 ]; then
        ./bin/tests
    else
        sudo ./bin/tests
    fi
}

print_help() {
    echo ""
    echo "用法: ./build.sh [选项]"
    echo ""
    echo "选项:"
    echo "  clean      清除可执行文件"
    echo "             编译可执行文件"
    echo "  run        执行所有单元测试"
    echo "  help       显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  ./build.sh clean        # 清除可执行文件"
    echo "  ./build.sh              # 编译可执行文件"
    echo "  ./build.sh run          # 执行所有单元测试"
    echo "  ./build.sh help         # 显示此帮助信息"
    echo ""
}

case "${1:-all}" in
    all|build)
        build_test
        ;;
    clean)
        clean_build
        ;;
    run)
        run_test
        ;; 
    help|--help|-h|-H)
        print_help
        ;;
    *)
        echo -e "${RED}未知选项: $1${NC}"
        print_help
        exit 1
        ;;
esac
