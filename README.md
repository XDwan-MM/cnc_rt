
### 版本管理

在 version.sh 文件中，

- 如果小功能变更/小问题修复，在 第三位 +1

- 如果大功能变更/大问题修复，在 第二位 +1


### 制品库

０　创建分支推送，可参考 template-ci.yml，创建流水线，在制品库下载对应压缩包（也可在feature分支手动生成）

１ 文件包括　bin config lib

２ 执行方式，在 cnc_rt_x.x.xxxx/bin 打开终端运行

sudo -E LD_LIBRARY_PATH=../lib/cncshm/bin ./rt_cnc

解释：以root权限，指定动态库在指定路径查找，执行rt_cnc


### 使用方法

0、更新子模块（如共享内存）

sudo rm -rf ./lib/cncshm/ （如报错文件已存在）

拉取当前版本

git submodule init && git submodule update --init --recursive

更新到最新版本（请勿轻易使用，除非确认需要更新到子模块最新版本）

git submodule init && git submodule update --remote


1、修改界面参数配置文件

1-1、测试平台（不靠墙那台）
逻辑轴0 的参数 电子齿轮比分母 设置为 1500000
逻辑轴1 的参数 电子齿轮比分母 设置为 1500000
逻辑轴2 的参数 电子齿轮比分母 设置为 8388608

1-2、测试平台（靠墙那台）
逻辑轴0 的参数 电子齿轮比分母 设置为 100000
逻辑轴1 的参数 电子齿轮比分母 设置为 100000
逻辑轴2 的参数 电子齿轮比分母 设置为 100000


2、修改主站配置文件

打开 master.c 文件，如果为测试平台2，请注释 #define PLATFORM_1


3、编译

清理编译（可选）sudo ./build.sh clean

sudo ./build.sh


4、编译并运行

启动人机界面 sudo ./cnc_hmi_v3

编译并运行实时　sudo ./build.sh


3、运行实时

cd bin
sudo ./rt_cnc


4、调试模式

cd bin
sudo ./rt_cnc_d




### 不同平台master和ENI文件配置

1、当更换平台时，需使用要更换的测试平台的master.c文件进行替换

- 9楼实验室第一台测试平台：speed.xml

- 9楼实验室第一台测试平台：gugao_io.xml

示例：

```C
int Master_Start() {
    char eni_path[512];
    snprintf(eni_path, sizeof(eni_path), "%s/%s", PROJECT_DIR, "config/ENI/gugao_io.xml");
    int error_code = ethercat_init(eni_path);
    if ( error_code < 0) {
        printf("[Warning] EtherCAT 启动失败，错误码为 %d ！\n", error_code);
        return -1;
    }
    return 1;
}
```


### PLC查看FG变量

１、参数　权限管理　系统管理员　登录　444444

２、诊断　状态显示　FG



### 常见问题解决

１、编译失败

先尝试sudo ./build.sh clean


２、库版本/编译器版本等问题，重新编译

sudo ./build.sh


3、sudo: ./build.sh：找不到命令

sudo chmod +x ./build.sh


４、时间在未来

make: 警告：文件“../shm.pro”的修改时间在未来 18348 秒后
/opt/Qt5.15.16/5.15.16/gcc_64/bin/qmake -o Makefile ../shm.pro CONFIG+=debug

sudo find ./* -type f -exec touch {} \;

5、./build.sh: 行 55: qmake: 未找到命令

sudo visudo

修改Defaults secure_path, 在最后添加实际qmake路径, 例子(需要自己修改最后的路径!!!):
Defaults        secure_path="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin:/home/wcf/QtLib/Qt5.15.16/5.15.16/gcc_64/bin"

ctrl+X 回车 保存退出.

6、按照5设置后，提示qmake不能找到lisence问题

找到qt企业版的lisence路径（找.qt-lisence文件在哪）
sudo ~/.bashrc
末尾添加一行内容（等号后面需要填写自己实际的.qt-lisence路径）
例如：export QT_LICENSE_FILE=/home/wcf/.qt-license

sudo visudo
添加Defaults项内容
Defaults        env_keep += "QT_LICENSE_FILE"
ctrl+X 回车 保存退出.


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

编译失败：
先尝试sudo ./build.sh clean

sudo ./build.sh 
cd bin && ./rt_cnc

