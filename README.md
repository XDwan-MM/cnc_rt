
### 运行

1、目录结构

```SHELL
|-- cnc_hmi*	  # 界面
|-- cnc_rt*       # 实时
|-- run.sh        # 启动脚本
```

2、启动方式

用法

```SHELL
./run.sh [选项]
```

选项

```SHELL
debug      # 运行调试模式

run        # 运行测试平台

help       # 显示帮助信息

hmi        # 只启动 HMI，进入界面后手工按 Ctrl+Enter
```

示例

```SHELL
  sudo ./run.sh debug      # 运行调试模式

  sudo ./run.sh run        # 运行测试平台

  sudo ./run.sh help       # 显示帮助信息

  sudo ./run.sh hmi        # 只启动HMI，进入界面后手工按 Ctrl+Enter
```

3、启动流程

正常流程：等待界面 → 界面初始化 → 实时初始化 → 进入主界面

异常流程：如果界面长时间处于等待界面，可能是主站启动失败，此时50s后无法进入，等待界面显示报警信息，测试阶段请关闭重启。

---

### 配置文件

如果需要**上机测试**（仿真模式随意），请注意配置文件的参数设置

1、修改界面参数配置文件

拷贝data和plc文件夹到cnc_hmi*中替换

- 测试平台1（不靠墙那台）

逻辑轴0 的参数 电子齿轮比分母 设置为 1500000

逻辑轴1 的参数 电子齿轮比分母 设置为 1500000

逻辑轴2 的参数 电子齿轮比分母 设置为 8388608

- 测试平台2（靠墙那台）

逻辑轴0 的参数 电子齿轮比分母 设置为 100000

逻辑轴1 的参数 电子齿轮比分母 设置为 100000

逻辑轴2 的参数 电子齿轮比分母 设置为 100000

- 2楼机床

逻辑轴0 的参数 电子齿轮比分母 设置为 1048576

逻辑轴1 的参数 电子齿轮比分母 设置为 1048576

逻辑轴2 的参数 电子齿轮比分母 设置为 1048576


2、修改运行程序

拷贝NC、TXT文件到 /NC/program 下

---

### 实时断点调试

在拥有cnc_rt*/, cnc_hmi*/和run.sh 的目录或者直接在 `cnc_rt/` 目录，新建终端：

1) 运行：
```SHELL
./run.sh hmi             # 只启动 HMI，进入界面后手工按 Ctrl+Enter
```
2) 在弹出的人机交互UI 手工按 Ctrl+Enter 启动HMI，一直挂着

3) 在vscode中，运行和调试工作栏中，启动gdb调试。

---

### 不需界面参数等数据写共享内存的单实时断点调试

如果想脱离界面，**只启动**实时进行断点调试：

1) 为了解除与界面的交互，请在 main.cpp 结束该部分程序段的注释
解释：该段模拟界面已发送启动实时的信号

```c
    // 如果想脱离界面进行调试，请解除注释
    // #ifdef DEBUG_MODE_ENABLED
    // setSysCommInt(SYS_START_RT, 1, 0);
    // #endif
```

2) 重新编译调试版本：sudo ./build.sh debug

3) 在vscode中，运行和调试工作栏中，启动gdb调试

---


### 共享内存

1、cnc_hmi 和 cnc_rt 的共享内存版本需保持一致。否则无法启动

2、连接数大于2，段错误

- ipcs -m：找到连接数最大（从右侧到左第3列），复制 shmid 号

- ipcrm -m <shmid>

**3、修改参数**

如果需要增删共享内存参数

#### 实时部分（负责人：许晓婷）

1、修改 lib/cncshm/api/gncdatadef 中的枚举类型

2、如果该参数为重启/复位生效！！！

- 修改 src/datadef/rt_data

- 修改 rt_update_reboot_valid_data 和 rt_upate_reset_valid_data


#### 界面部分（负责人：梁展鹏）

1、修改 cnc_hmi/conf/PARM_CON.json

2、修改 cnc_hmi/data/param.json


#### 更新配置

1、无误后，更新主仓库配置文件

2、修改共享内存版本号，实时和界面一起更新版本号

3、发邮件通知各组人员

---

### 版本管理

在 version.sh 文件中，

- 如果小功能变更/小问题修复，在 第三位 +1

- 如果大功能变更/大问题修复，在 第二位 +1

---

### 仓库管理

1、feature等保护分支，不要git push -f！！！

2、提交使用合并请求

---

### 制品库

０　创建分支推送，可参考 template-ci.yml，创建流水线，在制品库下载对应压缩包（也可在feature分支手动生成）

１ 文件包括　bin config lib ……

２ 执行方式，在 cnc_rt_x.x.xxxx/bin 打开终端运行

sudo ./cnc_rt

---

### 误差补偿数据采集到文件中，并在新终端界面打印显示

1、取消debug_mode.h中对应 #define ENABLE_COMP_DATA_LOG 的注释

```C
// ============ 补偿数据记录开关 ============
// 编译时开关：定义此宏可在 compute_error_comp() 中每个周期记录补偿数据到CSV文件
// 文件保存位置：与 run.sh 同级目录（即项目根目录 cnc_rt/）
// 取消注释以启用；或通过 qmake CONFIG 自动注入
#define ENABLE_COMP_DATA_LOG
```

2、实时观察补偿数据：在run.sh同级目录打开终端，运行
tail -f ./comp_data_log.csv

---

### PLC查看FG变量

１、参数　权限管理　系统管理员　登录　444444

２、诊断　状态显示　FG

---

### 更新子模块

更新子模块（如共享内存）

sudo rm -rf ./lib/cncshm/ （如报错文件已存在）

拉取当前子模块代码（将子模块的代码切换到主项目所记录的特定提交（而不是子模块的最新提交））

git submodule update --init --recursive

更新到最新版本（请勿轻易使用，除非确认需要更新到子模块最新版本）

git submodule init && git submodule update --remote

---

### 环境配置

linux 为 ubuntu22，可以使用真实linxu环境、虚拟机、wsl环境。

#### wsl环境

0、阅读培训教程，链接 (https://pan.gree.com/link/XBEaJVQXBC7xWeg/)

1、安装wsl

```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

2、重启

3、更新wsl，下载安装（https://pan.gree.com/link/2PdqqqWiOuuQCq7/），更新wsl版本，wsl2需要win10 22H2，低于这个版本更新不了只能用wsl1

4、下载ubuntu-wsl，链接 (https://pan.gree.com/link/eKPxhVxgiCSH24M/)

5、导入

```powershell
wsl --import Ubuntu22 D:\WSL\Ubuntu22 C:\Users\Admin\Downloads\
ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz
```

6、进入linux，wsl -l -v 可以确认安装结果

```powershell
wsl -d Ubuntu22
```

7、配置源，链接  (https://pan.gree.com/link/lelJdACFsKqNHFn/)

```bash
sudo cp sources.list /etc/apt
sudo apt update
```

8、创建普通用户

```bash
# 创建普通用户（这里用'gree'作为用户名，可以替换为需要的用户名，比如'zy'、'joey'）
adduser gree
# 将新用户添加到sudo组
usermod -aG sudo gree
```

9、设置默认账户，编辑 /etc/wsl.conf，添加下面内容，重启wsl生效

```
[user]
default=gree
```

10、配置界面显示，联云镜像下载MobaXterm，配置~/.bashrc，添加下面内容。自行检查，需要和MobaXterm的X11服务对上即可（软件右上角X图标，鼠标放上去有显示url，写入DISPLAY变量即可）。

```bash
# 配置显示
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
```

11、安装依赖包

``` 
sudo apt install \
apt-offline base-files base-passwd bash bsdutils btop build-essential ca-certificates ccache clangd dash diffutils findutils fonts-wqy-microhei fonts-wqy-zenhei git grep gzip hostname htop init language-pack-zh-hans libdebconfclient0 libgl1-mesa-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libnl-3-200 libnl-genl-3-200 libpoppler-qt5-dev libx11-xcb1 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 libxcomposite1 libxdamage1 libxfixes3 libxi6 libxkbcommon-x11-dev libxrender1 libxtst6 lld login mesa-common-dev micro ncdu ncurses-base ncurses-bin pkg-config screen sysvinit-utils tree ubuntu-minimal ubuntu-standard ubuntu-wsl
```

12、安装Qt5，链接 (https://pan.gree.com/link/cnOrBvColHNglbr/)

```bash
sudo chmod +x ./qt-enterprise-linux-x64-5.15.16.run
./qt-enterprise-linux-x64-5.15.19.run
```

安装密钥如下

```密钥
第一个空行：zby3 Gree
第二个空行：BG8RP-XQP-F4M-SCX-Y5FQH-KK8V7-6FF5
```

#### 虚拟机环境

1、安装vmware软件

2、安装虚拟机，链接 (https://pan.gree.com/link/G3N5lCmOAMEqSZa/)

3、参考上面换源、安装Qt

---

### 项目结构

```SHELL
|--bin				    # 可执行文件
|--config         # 配置文件
|--lib
|   |--cncshm		  # 共享内存库
|   |--errorcomp	# 补偿库
|   |--kinematic	# 运动库
|   |--parser		  # 译码库
|--src
|   |--alarm		      # 报警
|   |--common		      # 配置文件
|   |--compensation		# 补偿
|   |--datadef		    # 数据定义
|   |--hal			      # 硬件抽象层
|   |--master		      # 主站相关
|   |--motion		      # 运动相关
|   |   |--auto	      # 自动模式
|   |   |--manaul	    # 手动模式
|   |   |--spindle		# 主轴相关
|   |   |--emcmot	    # 路径规划和轨迹规划
|   |--plc            # plc相关
|   |--task			      # 事件相关
|   |--threads		    # 线程相关
|   |--utils		      # 工具类
|--main				# 程序入口
|--build			# 运行脚本
|--README			# 说明文档
```


**main.cpp**

初始化共享内存 → 初始化硬件抽象层 → 启动主站 → 创建信号量 → 创建线程


**motion/emcmot.cpp**

- emcmotPathPlan：路径规划

- emcmotTrajPlan：轨迹规划


**threads_controller.cpp**

- soft_plc_thread_func：PLC线程
- motion_controller_thread_func：运动线程
- task_controller_thread_func：任务线程
- path_plan_thread_func：路径规划线程
- pdo_thread_func：pdo线程，产生信号量

---

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


7、权限问题不能调试：

sudo visudo

行末添加：[用户名] ALL=(ALL) NOPASSWD: ALL

注意需要修改为自己的用户名，例子： gree ALL=(ALL) NOPASSWD: ALL
