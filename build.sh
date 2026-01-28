SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
echo "脚本所在目录：$SCRIPT_DIR"

# 编译 rt_inter
cd $SCRIPT_DIR
mkdir -p ./build
cd ./build
qmake ../RT_CNC.pro
make -j$(nproc)
if [ $? -eq 0 ]; then
    echo "RT_CNC 编译成功!"
else
    echo "RT_CNC 编译失败！"
    exit 1
fi
