重构实时进程版本


编译命令:
使用 CONFIG+=debug_mode 启用调试模式编译
使用 CONFIG+=release 或不加参数使用实时模式编译
调试模式编译: cd build/debug && qmake ../../RT_CNC.pro CONFIG+=debug_mode && make -j$(nproc)
实时模式编译: cd build/release && qmake ../../RT_CNC.pro && make -j$(nproc)
使用编译脚本可直接编译两个版本: sudo ./build.sh，有问题可以使用 ./build.sh help 查看帮助

权限问题不能调试：
1. sudo visudo
2. 行末添加：[用户名] ALL=(ALL) NOPASSWD: ALL
   注意需要修改为自己的用户名，例子： gree ALL=(ALL) NOPASSWD: ALL

运行命令：
1. 进入 bin 文件夹内
2. 输入命令: sudo LD_LIBRARY_PATH=../lib/cncshm/bin/:../lib/log/bin/ ./rt_cnc

生成compile_commands.json命令：
1. 修改 build.sh 中的make -j$(nproc) 。改为 bear -- make -j$(nproc)
2. 执行 sudo rm -rf build && sudo ./build.sh




### 键盘调试

1、配置

1. sudo visudo
2. 行末添加：[用户名] ALL=(ALL) NOPASSWD: ALL
   注意需要修改为自己的用户名，例子： gree ALL=(ALL) NOPASSWD: ALL

2、打开vscode，点击调试


3、通过按键模拟

╔══════════════════════════════════════════════════════════════════╗
║                     CNC 键盘控制监控                             ║
╠══════════════════════════════════════════════════════════════════╣
║                          按键说明                                ║
╠══════════════════════════════════════════════════════════════════╣
║ [模式] 1:JOG  2:快速  3:手轮  4:回零  5:编辑  6:自动  7:MDI      ║
║ [倍率] Q/A:进给+/-10%  W/S:快速+/-10%  E/D:主轴+/-10%            ║
║ [点动] U/J:+X/-X  I/K:+Y/-Y  O/L:+Z/-Z                           ║
║ [主轴] Z:正转  X:停止  C:反转                                    ║
║ [运行] Enter:运行  Space:暂停  R:复位  Esc:急停                  ║
║ [帮助] h/H:按键点位  p/P:系统信息                                ║
╚══════════════════════════════════════════════════════════════════╝
[============== CNC HMI ===================================]
[状态: 空闲] [进给: 100%] [快速: 100%] [主轴: 100%]
[译码行:   0][运行行:   0][程序完成: 否]
[急停: 否][复位: 否][运行: 否][回零: 否][移动: 否]
[事件数量: 0]
[===========================================================]





### 使用方法

1、编译

sudo ./build.sh


2、运行实时

cd bin
sudo ./rt_cnc


3、调试模式

cd bin
sudo ./rt_cnc_d




### 仓库管理

1、提交至当前分支，当前分支不受保护，不要git push -f！！！

2、提交使用合并请求

3、运行命令：

1. 进入bin文件夹内
2. 输入命令: sudo LD_LIBRARY_PATH=../lib/cncshm/bin/:../lib/log/bin/ ./RT_CNC



### 项目结构

```SHELL
|--bin				# 可执行文件
|--config         # 配置文件
|--lib
|   |--cncshm		# 共享内存库
|   |--errorcomp	# 补偿库
|   |--kinematic	# 运动库
|   |--log			# 日志存库
|   |--parser		# 译码库
|--src
|   |--common		# 配置文件
|   |--datadef		# 数据定义
|   |--hal			# 硬件抽象层
|   |--master		# 主站相关
|   |--monitor    # 模拟器相关
|   |--motion		# 运动相关
|   |   |--emcmot	# 路径规划和轨迹规划
|   |--plc        # plc相关
|   |--spindle		# 主轴相关
|   |--task			# 事件相关
|   |--threads		# 线程相关
|   |--utils		# 工具类
|--main				# 程序入口
|--build			# 运行脚本
|--README			# 说明文档
```



**main.cpp**

初始化共享内存 → 初始化硬件抽象层 → 启动主站 → 创建信号量 → 创建线程



**motion/emcmot.cpp**

1、emcmotPathPlan：译码 → 预处理

2、emcmotTrajPlan：读取抽象层数据 → 判断系统是否报错 → 判断运动模式 → 计算运动数据 → 补偿 → 检查超限位 → 更新抽象层数据 → 更新共享内存数据


**threads_controller.cpp**

- soft_plc_thread_func：PLC线程
- motion_controller_thread_func：运动线程
- task_controller_thread_func：任务线程
- path_plan_thread_func：路径规划线程
- pdo_thread_func：pdo线程，产生信号量




