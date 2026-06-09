#!/bin/bash
set -e

# ============================================================
# CNC_ErrorComp build script
# ============================================================
# Usage:
#   ./build.sh              Release build
#   ./build.sh debug        Debug build
#   ./build.sh release      Release build
#   ./build.sh clean        Remove build artifacts
#   ./build.sh test         Release build + run tests
#   ./build.sh debug test   Debug build + run tests
# ============================================================

BUILD_TYPE="Release"
RUN_TESTS=false
CLEAN_ONLY=false

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        debug|Debug|DEBUG)
            BUILD_TYPE="Debug"
            ;;
        release|Release|RELEASE)
            BUILD_TYPE="Release"
            ;;
        clean|Clean|CLEAN)
            CLEAN_ONLY=true
            ;;
        test|Test|TEST)
            RUN_TESTS=true
            ;;
    esac
done

# ----------------
# Clean
# ----------------
if [ "$CLEAN_ONLY" = true ]; then
    echo "Cleaning build artifacts..."
    rm -rf build bin lib
    echo "Clean finished."
    exit 0
fi

# ----------------
# Build
# ----------------
echo "========================================="
echo " Build type : ${BUILD_TYPE}"
echo " Run tests  : ${RUN_TESTS}"
echo "========================================="

mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build . --parallel

echo ""
echo "Build finished successfully."

# ----------------
# Test
# ----------------
if [ "$RUN_TESTS" = true ]; then
    echo ""
    echo "Running unit tests..."
    ctest --output-on-failure
fi
