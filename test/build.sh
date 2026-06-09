#!/bin/bash
set -e

# 获取脚本所在目录（确保在 test 目录执行）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "当前目录: $(pwd)"

clean_build(){
    # 强制删除 build（包括处理权限问题）
    if [ -d "build" ]; then
        echo "发现 build 目录，正在删除..."
        rm -rf build 2>/dev/null || sudo rm -rf build
    fi

    # 删除 bin（生成的可执行文件目录）
    if [ -d "../../bin" ]; then
        echo "发现 bin 目录，正在删除..."
        rm -rf ../../bin 2>/dev/null || sudo rm -rf ../../bin
    fi

    echo "已删除"
}

build_test(){
    # 重新创建并构建
    echo "创建新 build 目录..."
    mkdir build
    cd build

    echo "执行 qmake..."
    qmake ..

    echo "执行 make..."
    make -j$(nproc)

    echo "构建完成！"
}

run_test(){
    cd "$SCRIPT_DIR"
    sudo -E LD_LIBRARY_PATH=../lib/cncshm/bin ./bin/tests
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