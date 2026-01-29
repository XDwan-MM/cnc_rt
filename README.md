重构实时进程版本

提交至当前分支，当前分支不受保护，不要git push -f！！！
提交使用合并请求

编译命令:
使用 CONFIG+=debug_mode 启用调试模式编译
使用 CONFIG+=release 或不加参数使用实时模式编译
调试模式编译: cd build/debug && qmake ../../RT_CNC.pro CONFIG+=debug_mode && make -j$(nproc)
实时模式编译: cd build/release && qmake ../../RT_CNC.pro && make -j$(nproc)
使用编译脚本可直接编译两个版本: sudo ./build.sh，有问题可以使用 ./build.sh help 查看帮助

运行命令：
1. 进入 bin 文件夹内
2. 输入命令: sudo LD_LIBRARY_PATH=../lib/cncshm/bin/:../lib/log/bin/ ./rt_cnc

生成compile_commands.json命令：
1. 修改 build.sh 中的make -j$(nproc) 。改为 bear -- make -j$(nproc)
2. 执行 sudo rm -rf build && sudo ./build.sh