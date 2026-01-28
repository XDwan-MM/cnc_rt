重构实时进程版本

提交至当前分支，当前分支不受保护，不要git push -f！！！
提交使用合并请求

编译命令:
1. 执行 sudo ./build.sh，注意检查qmake是否被添加到环境变量

运行命令：
1. 进入 bin 文件夹内
2. 输入命令: sudo LD_LIBRARY_PATH=../lib/cncshm/bin/:../lib/log/bin/ ./RT_CNC

生成compile_commands.json命令：
1. 修改 build.sh 中的make -j$(nproc) 。改为 bear -- make -j$(nproc)
2. 执行 sudo rm -rf build && sudo ./build.sh