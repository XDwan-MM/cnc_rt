mkdir -p build
cd build
qmake ../shm.pro
make -j$(nproc)