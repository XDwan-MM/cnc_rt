# 清理 共享内存 构建目录
if [ -d "./build" ]; then
    rm -rf "./build"
    echo "  已清理: ./build"
fi

if [ -d "./bin" ]; then
    rm -rf "./bin"
    echo "  已清理: ./bin"
fi

#!/bin/bash
mkdir -p build
cd build
cmake ..
cmake --build .
