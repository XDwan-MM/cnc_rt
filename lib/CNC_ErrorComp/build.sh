#!/bin/bash
if [ "$1" = "clean" ]; then
    echo "Cleaning build and bin directories..."
    rm -rf build
    rm -rf bin
    echo "Clean finished."
else
    mkdir -p build
    cd build
    cmake ..
    cmake --build .
fi