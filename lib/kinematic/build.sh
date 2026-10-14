#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

if [ -d "$SCRIPT_DIR/build" ]; then
    rm -rf "$SCRIPT_DIR/build"
    echo "  已清理: $SCRIPT_DIR/build"
fi

if [ -d "$SCRIPT_DIR/bin" ]; then
    rm -rf "$SCRIPT_DIR/bin"
    echo "  已清理: $SCRIPT_DIR/bin"
fi

mkdir -p build
cd build
cmake ..
cmake --build .
