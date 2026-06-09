# CNC_ErrorComp

CNC 运动控制误差补偿库，为数控系统提供实时轴向误差校正。支持**垂直度补偿**和**螺距误差补偿**两种补偿模式，所有补偿输出经过平滑步进限制及 ±50μm 硬安全限幅。

库完全自包含（零外部依赖），`CompParams` 字段按实时控制器 `PAR_CMP_*` 枚举扁平排列，使用时逐字段手动赋值即可。

## 目录结构

```
CNC_ErrorComp/
├── include/                     # 公共头文件
│   ├── comp_common.h            # 通用定义：状态机枚举、限幅宏、角度转换常量
│   ├── comp_params.h            # 运行时参数结构体 (CompParams)，字段按 PAR_CMP_* 枚举扁平排列
│   ├── perp_comp.h              # 垂直度补偿 API
│   └── pitch_comp.h             # 螺距补偿 API
├── src/                         # 库源码 (纯 C)
│   ├── perp_comp.c              # 垂直度补偿：双项叠加 + 平滑过渡
│   ├── pitch_comp.c             # 螺距补偿：查表插值 + 平滑过渡
│   └── pitch_comp_table_init.c  # 补偿表生成：哨点设置 + 斜率预计算
├── tests/                       # 单元测试 (GTest / C++)
│   ├── CMakeLists.txt
│   ├── test_perp_comp.cpp       # 垂直度补偿测试
│   └── test_pitch_comp.cpp      # 螺距补偿测试
├── CMakeLists.txt               # 顶层 CMake 构建文件
├── build.sh                     # 一键构建脚本
└── README.md
```

## 参数结构体 (CompParams)

`CompParams` 按 `PAR_CMP_*` 枚举顺序扁平排列，字段命名规则为 `<补偿前缀>_<枚举后缀>`，与 gncdatadef.h 中的枚举常量精确对应：

| 字段 | 类型 | PAR_CMP 枚举 | 说明 |
|------|------|-------------|------|
| `bl_enable` | int | `PAR_CMP_BL_ENABLE` (0) | 反向间隙补偿类型 |
| `bl_value` | double | `PAR_CMP_BL_VALUE` (1) | 反向间隙补偿值 |
| `bl_rate` | double | `PAR_CMP_BL_RATE` (2) | 反向间隙补偿率 |
| `bl_value2` | double | `PAR_CMP_BL_VALUE2` (3) | 快移反向间隙补偿值 |
| `bl_mpg_value` | double | `PAR_CMP_BL_MPG_VALUE` (4) | 手摇反向间隙补偿值 |
| `pitch_type` | int | `PAR_CMP_PITCH_TYPE` (20) | 螺距误差补偿类型 |
| `pitch_start` | double | `PAR_CMP_PITCH_START` (21) | 螺距误差补偿起点坐标 |
| `pitch_num` | int | `PAR_CMP_PITCH_NUM` (22) | 螺距误差补偿点数 |
| `pitch_step` | double | `PAR_CMP_PITCH_STEP` (23) | 螺距误差补偿点间距 |
| `pitch_modulo` | int | `PAR_CMP_PITCH_MODULO` (24) | 螺距误差取模补偿使能 |
| `pitch_factor` | double | `PAR_CMP_PITCH_FACTOR` (25) | 螺距误差补偿倍率 |
| `pitch_table` | int | `PAR_CMP_PITCH_TABLE` (26) | 螺距误差补偿表起始参数号 |
| `squ1_enable` | int | `PAR_CMP_SQU1_ENABLE` (30) | 第1项垂直度补偿使能 |
| `squ1_input_ax` | int | `PAR_CMP_SQU1_INPUT_AX` (31) | 第1项垂直度补偿基准轴号 |
| `squ1_refn` | double | `PAR_CMP_SQU1_REFN` (32) | 第1项垂直度补偿基准点坐标 |
| `squ1_ang` | double | `PAR_CMP_SQU1_ANG` (33) | 第1项垂直度补偿角度 |
| `squ2_enable` | int | `PAR_CMP_SQU2_ENABLE` (40) | 第2项垂直度补偿使能 |
| `squ2_input_ax` | int | `PAR_CMP_SQU2_INPUT_AX` (41) | 第2项垂直度补偿基准轴号 |
| `squ2_refn` | double | `PAR_CMP_SQU2_REFN` (42) | 第2项垂直度补偿基准点坐标 |
| `squ2_ang` | double | `PAR_CMP_SQU2_ANG` (43) | 第2项垂直度补偿角度 |
| `stra1_input_ax` | int | `PAR_CMP_STRA1_INPUT_AX` (50) | 第1项直线度补偿基准轴号 |
| `stra1_type` | int | `PAR_CMP_STRA1_TYPE` (51) | 第1项直线度补偿类型 |
| `stra1_start` | double | `PAR_CMP_STRA1_START` (52) | 第1项直线度补偿起点坐标 |
| `stra1_num` | int | `PAR_CMP_STRA1_NUM` (53) | 第1项直线度补偿点数 |
| `stra1_step` | double | `PAR_CMP_STRA1_STEP` (54) | 第1项直线度补偿点间距 |
| `stra1_modulo` | int | `PAR_CMP_STRA1_MODULO` (55) | 第1项直线度取模补偿使能 |
| `stra1_factor` | double | `PAR_CMP_STRA1_FACTOR` (56) | 第1项直线度补偿倍率 |
| `stra1_table` | int | `PAR_CMP_STRA1_TABLE` (57) | 第1项直线度补偿表起始参数号 |

> 字段名与枚举名（去掉前缀后的小写形式）**逐字对应**，适配层 `comp_params_from_acmp()` 中每条赋值语句可直接用枚举名作为数组下标，零心智负担验证。

## 功能特性

### 1. 垂直度补偿
- 支持**双项叠加**：每轴可配置两个独立的垂直度补偿项，分别由不同基准轴驱动
- 补偿公式：`Comp = (RefPos - RefPoint) × (Angle × 10⁻⁶)`
- 自补偿屏蔽：基准轴等于当前补偿轴时返回 0，防止逻辑死循环

### 2. 螺距误差补偿
- **查表线性插值**：基于预定义的测量点表，在区间内进行线性外推
- **单向 / 双向**补偿模式：单向使用正方向补偿表，双向根据速度方向自动选择正/反向
- 游标加速定位：运行时通过游标快速定位到当前指令位置对应的插值区间
- 零速锁定输出：速度接近零时保持上一时刻输出，防止微小抖动导致补偿跳变

### 3. 安全机制
| 机制 | 描述 |
|------|------|
| **首次运动入场** | 上电后轴未运动前，补偿输出强制为 0；检测到速度超过阈值后才从 0 平滑入场 |
| **步进限制** | 参数变更或补偿开关时，输出以固定步长逐步爬升/下降，防止机床冲击 |
| **±1000μm 硬限幅** | 单次补偿输出绝对值不超过 `MAX_COMP_LIMIT_PER` (1.0mm) |
| **三态机管理** | IDLE → RAMPING → ACTIVE 状态切换，保证所有过渡均经过平滑处理 |

## 快速开始

### 前置条件

- CMake >= 3.16
- C11 编译器 (GCC / Clang / MSVC)
- Google Test >= 1.14 (用于单元测试，可选)

```bash
# Ubuntu / Debian 安装 GTest
sudo apt install libgtest-dev
```

### 构建

```bash
# Release 构建
./build.sh

# Debug 构建 + 运行测试
./build.sh debug test

# 清理
./build.sh clean
```

等价的手动 CMake 命令：

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
ctest --output-on-failure
```

### 安装

```bash
cd build
cmake --install . --prefix /usr/local
```

### 集成到你的项目

```cmake
find_package(GTest REQUIRED)  # 如需构建测试
add_subdirectory(path/to/CNC_ErrorComp)
target_link_libraries(your_target PRIVATE CNC_ErrorComp)
```

## API 使用示例

### 垂直度补偿

```c
#include "perp_comp.h"

PerpCompState state;
perp_comp_init(&state, 0.0005);     // 步进限制 0.5μm/周期

CompParams par_cmp = {0};
par_cmp.squ1_enable    = 1;
par_cmp.squ1_input_ax  = 0;         // 基准轴 X
par_cmp.squ1_ang       = 50.0;      // 角度 50μm/m

double axis_pos[3] = {200.0, 0.0, 0.0};
double vel = 100.0;                 // 进给速度 mm/s

// 每伺服周期调用一次
double comp = calculate_perpendicularity_total(
    &par_cmp, &state, 1, axis_pos, vel);  // cur_ax_idx=1 (Y轴)
```

### 螺距补偿

```c
#include "pitch_comp.h"

PitchCompState state;
pitch_comp_init(&state, 0.0005);

pitch_comp_table_t table = {0};
double points[]   = {0.0, 10.0, 20.0};
float  fwd_err[]  = {0.0f, 0.005f, 0.010f};
float  rev_err[]  = {0.0f, -0.003f, 0.008f};
generate_pitch_comp_table(&table, points, fwd_err, rev_err, 3);

CompParams par_cmp = {0};
par_cmp.pitch_type   = 1;           // 单向补偿
par_cmp.pitch_factor = 1.0;

// 每伺服周期调用一次
double comp = calculate_pitch_compensation(
    &state, &par_cmp, &table, pos_cmd, vel_cmd);
```

## 编码风格

项目使用 `clang-format` (基于 LLVM 风格，4 空格缩进)，运行：

```bash
clang-format -i src/*.c include/*.h tests/*.cpp
```
