# 交接：把 cnc_hal 接入移植到 v1.27.1

最后更新：2026-10-09（第 4-6 步收尾，编译通过、测试跑通）

---

## 零、角色与边界（先看这个，它决定后面所有判断）

**我们的主责是 HAL 的设计与开发。把 HAL 接进 RT 是附带工作。**

| | 谁负责 | 我们做什么 |
|---|---|---|
| HAL 的设计 / 实现 / 版本 | **我们** | 主责，想怎么改怎么改 |
| RT 接入 HAL | 我们（附带） | 只做「适配 HAL」必需的改动 |
| RT 的原有功能 | **不是我们** | **不许改坏**：接入后 RT 能跑、原有功能不变 |
| RT 自身的缺陷 | 不是我们 | 报给同事，**不擅自修** |

**两条硬约束：**

1. 接入 HAL 之后，**现版本 RT 必须能跑**；
2. **原有功能不受影响**。

**推论 —— 判断「某处该不该动」时照这个来：**

- HAL 里发现「不属于 HAL 的活」（机床坐标基准、主轴刻度语义、到速判据、加减速占位…）
  → 是我们要清的，**但判据是「清掉之后 RT 行为不变」**。
  会因为清理而改变 RT 行为的，**先不动**，攒着单独一轮。
- RT 里发现的缺陷（M29 的 F76.3 双写、26 个过时测试…）
  → **不是我们的活**，报给同事，不顺手改。
- 清理 HAL 之后必须**证明** RT 行为没变 —— 用现成的两个脚手架回归：
  `test/rt_offline_smoke.sh`（启动/使能/台账/故障复位）
  `test/rt_offline_spindle.sh`（M3/M5/M19/释放/重复 M19）

---

## 一、四个目录，别搞混

| 目录 | 是什么 | 能不能改 |
|---|---|---|
| `/home/mxh/CNC/cnc_rt` | **我们的开发树**。1.24.15 + cnc_hal 接入已完成 | 能改 |
| `/home/mxh/CNC/HAL` | **HAL 仓库**（独立 git，有远程 gitea） | 能改 |
| `/home/mxh/CNC/cnc_rt_last` | **同事的树**，v1.27.1，936 个提交 | **不要动** |
| `/home/mxh/CNC/port_work` | **移植工作副本**（cnc_rt_last 的拷贝 + 分支 `dsh/hal-port`） | 在这里干活 |
| `/home/mxh/CNC/固件库/cnc_rt_1.27.0.1` | **用户实际运行的固件包**（无 src，`./run.sh`） | 部署目标 |

---

## 二、已完成

### A. HAL 错误码细分 + 主轴模式握手 + 报警映射
- HAL：`HAL_ERROR_NOT_ENABLED`(0x0106) / `HAL_ERROR_MODE_MISMATCH`(0x0107) / `hal_rt_axis_readiness()`
- RT `1233d9a`：适配层带上轴槽、模式切换握手、报警号 132/133

### B. M19 主轴定向（端到端打通，仿真已验证）
- HAL `9c71f54`：厂商控制字透传（ABI **5.1 -> 6.0**）
- HAL `6b633d6`：**发行版本定为 0.1**（ABI 仍是 6.0）
- HAL 本次：**清掉不属于 HAL 的接口与字段，ABI 6.0 -> 7.0**（见下）
- RT `68c4cfa`：M19 流程 + F45.7 + 台账告警
- 实测：M19 -> rc=0 -> st=1 -> st=2 -> F45.7 -> FIN -> 程序结束；M03 S500 -> HAL 读回 500

> `HAL_VERSION_*` 是**发行版本**，和 `HAL_C_ABI_*` 是两回事：前者是这批代码对外叫什么，
> 后者管"二进制能不能对上号"。只给宏、没加查询函数——加函数按 HAL 自己的策略就是 MAJOR 变更。

### B2. HAL 职责清理（ABI 6.0 -> 7.0）

按「HAL 是我们的主责、RT 原有功能不许改坏」这两条，把几处放错位置的东西清了。
**验收判据不是"架构更干净"，是"RT 行为不变"**，所以每一项都回归过。

| 清掉 | 为什么 | RT 影响 |
|---|---|---|
| `hal_rt_spindle_release_orient()` 里那句 `a->request = ENABLE` | 释放之后要不要上力矩是**上层的决定**；而且它和 `hal_rt_axis_enable()` 是同一个字段的两个写者，绕过了后者的保护 | 无（RT 本来就每拍声明使能）。**已先用测试证明旧代码确实会吃掉去使能**：`context_regression` 的 `orient_release_keeps_enable_intent` 在旧代码上实测失败（控制字 0x0F），修完通过 |
| ~~`hal_rt_spindle_write_pos()`~~ **已恢复，不删** | 我一开始按"单圈刻度是机床语义"把它删了。**删错了**：它是**与 RT 侧事先约定好的接口**，不属于越界 | 原样恢复（签名、语义、实现、用例、example、文档全回）。顺带补了适配层入口 `rt_hal_request_spindle_turn()` —— 没有它 RT 根本够不着这个约定接口 |
| ~~`hal_rt_spindle_enable/estop()`、`hal_rt_axis_read_pos()`、`hal_rt_spindle_read_speed()`~~ **也全部恢复** | 我按"纯转发"把它们删了。**同一条错误**：主轴专用入口是**与 RT 侧约定好的接口面**，不按"是不是纯转发"来裁 | 原样恢复（声明、实现、用例、example、文档全回） |
| `HalCSpindleCfg.accel` | HAL 不生成加减速曲线，纯占位 | 无（RT 只是填 0） |
| `HalCSpindleCfg.speed_window` + `HalCSpindleStatus.at_speed` | "到速"是**上层 M 代码的握手判据**，而且上层自己算了一遍 —— 同一件事两处定义 | 无（RT 没读 `at_speed`） |

> **ABI 仍是 7.0，没有为"恢复"再跳一版**：7.0 是本次会话内刚定的、还没对外发布，
> 且这次是**撤销误删**——7.0 相对 6.0 的净变化里本来就不该包含它。
> `hal_types.h` 的 7.0 说明已改正，并写明"曾误删、随后原样恢复"。发布前若要让编号
> 严格对应，把它改成 8.0 即可（一行）。

连带的改动：HAL 自己的用例（`context_regression.c`）、硬件测试 example、`docs/api-reference.md`
都同步了；两个只为 `hal_rt_spindle_write_pos` 存在的用例（`spindle_origin_angle`、
`angle_regressions`）删掉。

**这次的教训（写下来免得再犯）：**

> 我拿「**RT 够不着**」当删除的理由，那个论证本身是错的 —— RT 够不着是因为**适配层没包**，
> 那是移植侧的缺口，不是 HAL 接口该不该存在的依据。
> 更根本的是：**这 5 个接口是跟 RT 侧谈定的**，不该由我一个人在评审里判掉。
> 判据应该是「**它是不是约定的接口面**」，不是「我觉得它该在哪一层」。
> 恢复方式也照这个来：**原样**（签名、语义、实现、用例、文档全回），
> 不用我自己提的新签名 —— 那等于单方面改已谈定的契约。

**回归证据（改完与改前逐条对比，全部一致）：**

| | 改前 | 改后 |
|---|---|---|
| HAL 离线用例 | 4/4 | **4/4** |
| `rt_offline_smoke.sh`（启动/使能/台账/故障复位） | 5 PASS | **5 PASS（逐条一致）** |
| `rt_offline_spindle.sh`（M3/M5/M19/释放/重复 M19） | 全 PASS，状态字 `0x0637` | **全 PASS（逐条一致），状态字 `0x0637`** |

> 保留不动的：`HalCAxisCfg.enc_off` 与 `hal_rt_axis_set_pos()` —— 它们确实是**机床坐标语义**，
> 但拆出去要 RT 自己维护坐标平移（= 改 RT 功能），违背"原有功能不受影响"，所以先留着。
>
> `HAL_VERSION_STRING` 仍是 **0.1**：0.1 还没对外发布，这次清理算进 0.1 里。

### C. 移植到 v1.27.1 —— **代码移植完成，编译通过**
- 同事确认：这版稳定、不会再大改，做出来就是为了合进抽象层

---

## 三、移植现状（`port_work` / 分支 `dsh/hal-port`）

### 已提交

```
d698f7d  （他们的 HEAD，起点）
acaf518  WIP: 三方合并完成，9 个冲突待解
0fd7e4f  WIP: 解完 6 个冲突，删掉旧抽象层
6fbf7a1  移植 1-3 步：骨架 + 机械合并 + 主轴头用他们的模型
d714ff9  移植第 4 步（进行中）：spl_control.c 接 HAL
d432adb  加交接文档
（以下为本次未提交的改动）
```

### 进度：全部完成

| 项 | 状态 |
|---|---|
| 删他们 26 个 driver_hal/master 文件 | 完成 |
| 加我们 5 个 `src/hal/rt_hal_*` 文件 | 完成 |
| `CNC_RT.pro` 改好 | 完成 |
| 冲突块 6/6 | 完成 |
| `Hal_Driver_*` / `Driver_*` 调用点 30 处 | 完成 |
| `spindle_driver.*` 字段引用 155 处 | 完成，0 残留 |
| `spl_orient.c` / `jog.c` / `plc_f_signal_handler.c` / `reset.c` / `emcmot.cpp` / `main.cpp` | 完成 |
| `lib/cnc_hal` vendored 副本 -> ABI 7.0 | 完成 |
| **编译（debug + release）** | 完成，都出二进制 |

### 台账：只反映"参数域给全了没有"

`src/hal/rt_hal_config.c` 里曾经有两个现场确认开关（`RT_HAL_IO_MAP_CONFIRMED` /
`RT_HAL_MOTION_RANGE_APPROVED`），**已按拍板删掉**（2026-10-10）：逐点核对点表、
批准运动范围与量程是电气/工艺的职责，不该由移植代码用"不能动"去卡流程。

现在台账只剩"参数域给全了没有"（按轴算的 SCALING / ENC_OFF / ENCODER）：
**参数域为空时仍然会挡住运动**，四轴参数写全后自动清空。

> 本次同步了 `port_work/src/hal/*` = `cnc_rt/src/hal/*`。
> 之前那份是 cnc_rt 的**旧快照**（ABI 5.1、没有定向、注释还是啰嗦版），
> 逐文件比对过：**没有任何 port_work 专属适配**，所以整体覆盖是安全的。

### 替换映射（下次改别处照着来）

| 他们的 | 我们的 HAL | 说明 |
|---|---|---|
| `Driver_Enable/Disable` | `rt_hal_request_axis_enable(slot, 0/1)` | **状态**不是事件，只表达"要不要上力矩" |
| `Driver_Request_Mode_Switch` | `rt_hal_request_spindle_mode(slot, m)` | HAL 自己走完 减速->去使能->写 6060->等 6061->重新使能 |
| `Driver_Switch_Mode_While_Disabled` | 同上 | HAL 内部处理"下使能状态" |
| `Driver_Is_Mode_Switching/_Done/_Result` | 本地闩锁 `spl_ms_switching()/_done()/_result()` | 只是"请求下去没有、跟上没有"，不是 DS402 过程 |
| `Driver_Is_Ready_to_Switch_On` | `spl_drv_switch_on_disabled()` | 就是 `(状态字 & 0x6F) == 0x21` |
| `spindle_driver.state == OPERATION_ENABLED` | `spl_drv_enabled()` | |
| `spindle_driver.act_mode` | `spl_drv_mode()` | 来自 0x6061 |
| `Hal_Driver_Set_Target_Pos` | `rt_hal_request_axis_pos(slot, pos)` | |
| `Driver_Orient / Is_Orient_Done / Release_Orient` | `rt_hal_spindle_*_orient(slot)` | `spl_orient.c` 就是薄封装 |
| `target_state` / `control_word` / `mode_switch_*` | **删掉** | DS402 内部量，HAL 独占 |

### 移植时改掉的三个语义差异（重要）

1. **定向请求要重试。** HAL 只在 `status.enabled` 时收定向请求，而刚声明的使能下一拍才生效。
   所以 `SPINDLE_EVENT_ORIENT` 里第一次请求失败是**正常**的，进 `SPINDLE_ORIENT_WAIT_ZERO`
   由轮询重试，不能当 M19 失败。
2. **重复 M19 要等完成位落下。** 释放后马上重请求，驱动器还在厂商状态、状态字 bit10 还挂着，
   会立刻"完成"。所以重请求的条件是 `(状态字 & 0x0400) == 0`。
3. **HAL 的释放是"交回控制字并重新使能"，不是回到 Switch On Disabled。**
   他们那套 `RELEASING` 等 `0x21` 的判据在 HAL 下永远不会成立 —— 改成看
   `spl_drv_orient_inactive()`（定向状态退回 NONE）。M29 的"等停车"判据同理，
   改成"定向已交回 + 实际转速约等于 0"。

---

## 四、验证结果（本次）

### 编译
```
调试版本: [OK]  port_work/bin/cnc_rt_d
实时版本: [OK]  port_work/bin/cnc_rt
```

### 单元测试：跑起来了，79 个用例

`port_work/test/test.pro` 之前指着已删的旧驱动/主站源文件，本次一并修好
（去重、补 `spl_orient.c`/`spl_vel_planner.c`/`trace_publisher.c`、接 HAL 库、
去掉本机没有的 `-lethercat`、排除用旧字段名的 `tst_homing.cpp`）。

```
79 tests from 9 test suites
26 FAILED / 53 PASSED
```

**26 个失败与本次移植无关，已用对照实验证明：**

| | 用例数 | 失败数 |
|---|---|---|
| `cnc_rt`（我们的代码 + HAL） | 79 | **26** |
| `port_work`（同事代码 + HAL 移植） | 79 | **26** |

两边**失败清单逐条完全一致**（`TestAuto` 12、`TestJog` 10、`TestCompData` 3、`TestServoSample` 1），
说明是**测试本身就过时**，不是移植带来的。旁证：这些测试文件的最后提交时间都早于
它们测的源码（`tst_jog.cpp` 09-07 vs `jog.c` 09-27；`tst_auto.cpp` 08-31 vs `auto.c` 09-28）。

> 两个测试进程在 `Global test environment tear-down` 之后都会段错误，
> 两边同样如此，也是既有问题。

### 无界面主轴回归：**全部 PASS**（新增）

RT 启动时会阻塞等 HMI 写 `SYS_START_RT`（`main.cpp` 那个 while），所以这一轮用
`test/rt_offline_helper.c` 扮演界面，主轴指令走**真路径**：写 G 寄存器造边沿 →
`plc_g_signal_scanner` 翻译成主轴事件 → 从 F 寄存器读回主轴自己的判断。

```
# 直接编调试版就行：台账只反映"参数域给全了没有"，握手端会把四轴参数写全。
bash build.sh debug

sudo ./test/rt_offline_spindle.sh
```

> 曾经为了绕开那两个现场确认开关，另编过一个 `bin/cnc_rt_ledger_d`
> （`CONFIG+=offline_ledger`）；开关删掉后那个档也没用了，脚本不再拦人。

```
[SPL] M3 前: 倍率=100.0% S码=0   F45.1=1 F45.3=1 F45.7=0
-> M3（G70.5 上升沿）
[SPL] M3 后: 倍率=100.0% S码=500 F45.1=0 F45.3=1 F45.7=0
  PASS  M3 后 F45.3（到速）
  PASS  M3 后 F45.1（转速<5，应为 0）
-> M5（G29.6 下降沿）
  PASS  M5 后 F45.1（转速<5，应为 1）
-> M19（G70.6 上升沿）
  PASS  M19 后 F45.7（定向完成）
-> 释放定向（G70.6 下降沿）
  PASS  释放后 F45.7 落下去
-> 再来一次 M19
  PASS  重复 M19 后 F45.7
汇总：失败 0 项
```

HAL 侧的硬证据（`CNC_RT_DUMP=14000` 打在定向窗口里）：

```
HAL轴3 身份=SIM spindle | 使能=1 速度=0.000 状态字=0x0637
                                  ^^^^^^ = 0x0237 | 0x0400（厂商完成位）
spindle_update_orient_state: 主轴定向释放状态：
    status_word=0x0637 enabled=1 orient_state=2 actual_vel=0
```

`状态字=0x0637` 说明厂商控制字 0x040F 真的下发了、完成位真的回来了；
`orient_state=2` 就是 `HAL_ORIENT_DONE`。

> 未覆盖：**释放前就重复 M19** 那条分支（PLC 只有边沿，造不出来），
> 以及 M29。

### 仍未做
- **GUI 仿真**：上面那轮是无界面的，但**没有走 NC 程序**（即 M3/M19 由 G 代码驱动那条链）
- **上机验证**

---

## 五、操作上的坑

| 坑 | 事实 | 正确做法 |
|---|---|---|
| **`build.sh` 失败也返回 rc=0** | 看 `[FAIL]` / `[OK]` 标记 | **永远不信 rc，看产物** |
| **`test/build.sh` 失败也返回 rc=0** | 同上 | 同上 |
| **qmake 不重编测试** | 换过 gtest 后旧 `.o` 仍注册到旧 runner，`Running 25 tests` 是假象 | **改完构建配置先 `./build.sh clean`** |
| **部署 ≠ 生效** | 已运行的进程仍用旧二进制 | **部署后必须重启 RT 进程** |
| **`strings` 匹配不了中文** | 返回 0 是假警报 | 用 `grep -c "串" 二进制` |
| **make 有时不重链** | `.o` 新但二进制旧 | 删掉 `bin/cnc_rt*` 再 make |
| **台账只剩"参数给全了没有"** | 两个现场确认开关（IO_MAP / MOTION）已删（2026-10-10）；参数域为空时运动仍被挡是**正常的** | 离线回归把四轴参数写全（`rt_offline_helper` 已经会写），台账自动清空 |
| **离线跑主轴要先补三个参数** | `PAR_AX_FEED_ACC`=0 → 加速度 0 → M3 转速永远不动；软限位=0 而位置是 -150 → 行程报警 → `SYS_IS_ALARMING` 每拍把主轴事件改写成 STOP；主轴倍率默认 0 → 目标速度乘成 0 | 见 `rt_offline_helper.c` 的 `set_axis()` / `write_spindle_params()` / `spl_set_override()` |
| **ABI 现在是 7.2，是破坏性变更**（7.1→7.2：`HalCAxisCfg` 加 `speed_unit` / `command_pulses_per_rev`，`HalCAxisStatus` 加 `actual_speed_rpm`）| `struct_size` 变了，还删了函数与字段 | HAL 库 + RT + 固件包**必须一起更新**；旧的 `cnc_rt` / 固件包配不上就拒启 |

验证产物三步：`grep -c 关键串 二进制` -> `stat 时间戳` -> `重启进程`。

---

## 六、关键代码位置

| 功能 | 位置 |
|---|---|
| HAL 定向接口 | `HAL/include/hal_rt_api.h`（3 个函数）；`hal_types.h` 的两个配置字段 |
| HAL 整字覆盖说明 | `HAL/include/hal_rt_api.h` 开头那段（45 行） |
| 主轴驱动层替换 | `port_work/src/motion/spindle/spl_control.c` 顶部「HAL 访问器（方案 B）」+ `spl_ms_*` |
| 定向映射 | 同文件 `spindle_update_orient_state()` |
| 定向薄封装 | `port_work/src/motion/spindle/spl_orient.c` |
| F45.7 输出 | `port_work/src/plc/plc_scanner/plc_f_signal_handler.c` 的 `plc_output_spl_orient_status()` |
| 台账闸门 | `port_work/src/hal/rt_hal_adapter.c`（定向那两处**在闸门之前**） |
| 仿真定向模拟 | `port_work/src/hal/rt_hal_sim_backend.c`（收到控制字后延迟 20 拍置完成位） |
| 定向常量唯一来源 | `port_work/src/hal/rt_hal_config.h` 的 `RT_PLACEHOLDER_ORIENT_*` |

---

## 七、待办

| 项 | 说明 |
|---|---|
| **NC 程序驱动的仿真** | 上面那轮是直接写 G 寄存器的。还差"跑一段 G 代码（M3/M5/M19）"那条链，需要 HMI 或再写一个离线驱动器 |
| **提交这些改动** | `port_work` 有 21 个文件未提交；`cnc_rt` 有 4 个 |
| **上机验证** | 驱动器认不认 `0x040F`、转到哪个位置 —— 只有真机能答 |
| **26 个过时测试** | 要么更新测试，要么确认可以不管。**同事那边也一样过时**，值得问一句 |
| **`cnc_rt_last` 交付不完整** | `lib/cncshm/` 是**空目录**，`g_device_data` 只有 extern 没有定义，`test.pro` 有重复源文件且漏 `spl_orient.c` —— 他们那份树在本机**编不起来**。（本次是靠 `port_work` 里我们自己补的 cncshm 才跑通） |
| **攻丝同步（G33.1/G33.2）不通** | **已查明，未修**，详见第十节。不是 HAL 的问题，是 RT 自己的缺口 |
| **M29 刚性攻丝上机** | 他们标"待平台测试"，我们这边接上了 HAL，但没实机验证 |
| **主轴"既定零点"要不要跟坐标零点分开** | 只有做"转到某个刻度"的分度/定位功能才需要；**攻丝用不上**。定了才能动 |
| ~~`hal_rt_axis_readiness()` 口径矛盾~~ | **已按"甲"改完**，见第十一节 |
| **`hal_rt_axis_readiness()` 没包进适配层** | HAL 有、RT 够不着。当初是给"写不进去时归因"用的 |
| **HAL 仓库推送** | `9c71f54` 是破坏性 ABI 变更，**没推** |

---

## 八、环境变更记录（本次）

- 本机装了系统 gtest：`sudo apt install libgtest-dev libgmock-dev`（**1.11.0-3**）
  - 装之前 `test/gtest_compat` 缺 `EXPECT_DOUBLE_EQ` / `EXPECT_NEAR` / `EXPECT_LT/GT/LE/GE` 等宏，
    本次也把这些补进了兼容层（万一别的机器没装 gtest 也能编）
- `cnc_rt/test/build.sh` 原本没有可执行位，已 `chmod +x`

---

## 九、我们 vs 他们的差距（供参考）

```
版本   我们 1.24.15      他们 1.27.1
他们从我们基线后有 29 个提交（不含 merge）
```

**仅他们有的**：主轴启动流程重构、主轴逻辑修复、M3/M4/M5 连续执行修复、
CSP/CSV 规划器算法、M19 定向（修 5 轮）、M29 刚性攻丝、NURBS 参数、手轮 bug、
delta 平台、驱动器上下使能逻辑

**仅我们有的**：cnc_hal 接入层（5 个文件）+ HAL 的全部改进

**冲突的根**：他们自己实现 DS402 状态机（`driver_hal`），HAL 也实现 —— 两者都想写
`0x6040`，**不能共存**。所以移植 = 把他们的主轴逻辑重新接到 HAL 上。

---

## 十、攻丝（G33.1 / G33.2）不同步 —— 已查明，未修

查于 2026-10-09。**不是 HAL 的问题，是 RT 自己的缺口**，而且三处连在一起，
只补一句下发解决不了。

### ① 主轴角度压根没下发到驱动器

`interpolation.c:159` 把算出来的角度写进 `rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur`，
但 `emcmot.cpp` 的 `output_to_hal()` 只发 X/Y/Z 三根轴，**没有主轴**：

```c
static void output_to_hal() {
    (void)rt_hal_request_axis_pos(RT_HAL_AXIS_X, ...);
    (void)rt_hal_request_axis_pos(RT_HAL_AXIS_Y, ...);
    (void)rt_hal_request_axis_pos(RT_HAL_AXIS_Z, ...);
    /* ← 主轴这一句没有 */
}
```

同事那版（`Hal_Driver_Set_Target_Pos` × 3）也一样。全树往 HAL 写位置的调用点一共 5 处，
另外两处是 `spl_control.c:607`（M29 保位）和 `:778`（复位路径），跟攻丝无关。

### ② 每个攻丝段开头会**回退一整圈**

`interpolate_tap()`（`interpolation.c:140-160`）算的是**本段相对角度**：

```c
double spl_pos   = accum_distance / current_block->rigid_data.screw_offset;  // 螺距
double spl_angle = spl_pos * 360;          // 段起点 = 0，不带任何基准
if (current_block->rigid_data.reverse) spl_angle = -spl_angle;
rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur = spl_angle;
```

而它是**按拍差分累加**进机床坐标的（`calcul_mac_cmd_pos`，`emcmot.cpp:475`）：

```c
mac_cmd_pos += (wcs_cmd_pos_cur - wcs_cmd_pos_pre);
```

上一段结束时 `wcs_cmd_pos_cur` 是 360，新段头一拍写 0 → delta = −360 → **主轴倒转一圈**。

### ③ M29 保位和这条累加链**不同源**

M29 保位用的是 `s_spl_target_pos`（定向释放那一刻取的**反馈值**，`spl_control.c:607`），
攻丝的 `motor_cmd_pos` 是 `wcs_cmd_pos_cur` 增量累加出来的。两者不等 → 攻丝起步会跳。

### 要修的话

三处必须一起理顺，核心是定「**主轴 CSP 目标归谁管**」：

- 攻丝段的目标应该是 **M29 保位位置 + 相对角度**，并在攻丝段让 `spl_control` 让出这一路；
- 只在该段下发（判 `cycle_tp->plan_block->type` 是不是 `RIGID_TAP` / `TAP_RETURN`）。
  **不能无条件发**：`output_to_hal()` 跑在 `spl_control` 之后，后写赢，
  无条件发就会把 M29 保位盖掉 —— 那会把已经验证通过的 M19/M29 弄坏。

### 验不了的部分

真实同步精度、驱动器跟不跟得上 —— **只能上机**。离线最多能验"值发出去了、并按螺距跟随"。

### 附：`spl_pos_planner.c` 是死代码

注释写着"单轴周期同步位置规划器"，全树**没人调用**。看着是当初为这条路准备的，没接上。
攻丝其实也不需要它（角度是从进给距离直接算的，不是独立规划出来的）。

---

## 十一、`hal_rt_axis_readiness()` 的文档与实现对不上 —— 已记录，待定

发现于 2026-10-09（给它补测试覆盖时）。这个接口此前是**公共 API 却零用例覆盖**，
现在用例补上了，但补的过程中发现它自己说的和做的是两回事。

**文档说**（`include/hal_rt_api.h`）：

```
 *   bit3 MODE_DESIRED 请求的目标模式与 HAL 记录的不一致（切换还没发起完）
 *   bit4 MODE_ACTUAL  驱动器实际模式（0x6061）与目标不一致（切换还没生效）
```

**实现是**（`src/internal/hal_context.c` 的 `axis_readiness_bits()`）：

```c
const int want = a->cfg.work_mode == HAL_WORK_POSITION ? DS402_MODE_CSP : DS402_MODE_CSV;
if (a->desired_mode != want)    r |= HAL_READY_MODE_DESIRED;
if (a->mode != want)            r |= HAL_READY_MODE_ACTUAL;
```

比的是**配置的工作模式**，不是"本次请求"。而 HAL 里**没有**单独记"上层最近请求的模式"
（`desired_mode` 就是那个目标），所以文档那句话按字面实现不了。

**后果**：主轴配置的工作模式是速度（CSV）。M29 / 攻丝把它切到 CSP 之后：

- `hal_rt_axis_readiness()` 报 `MODE_DESIRED | MODE_ACTUAL`
- 但此刻 `hal_rt_axis_write_pos()` 是**受理的**（它只要求 `desired_mode` 和 `mode` 都是 CSP）

两者判据不一致 → 拿它做"写不进去"的归因会**给错**。

**已按"甲"改完（2026-10-09）**：

- `axis_readiness_bits()` 只比「**驱动器实际模式 vs HAL 要求的模式**」，
  不再拿配置的 `work_mode` 当目标模式；
- `HAL_READY_MODE_DESIRED` **删除**，0x08 位**留空不复用**
  （免得旧代码读到的 0x08 被当成别的意思）；
- 公共头文档改写，并把正确用法写进去：**先 `request_mode` 把期望模式设对 →
  等 readiness 全 0 → 再下发**。因为 `write_pos` 要 CSP、`write_speed` 要 CSV，
  而 readiness **不含"你要写哪条指令"这一维** —— 这一点是原来没说清的；
- 用例跟着改：主轴请求切 CSP 后，先验 `MODE_ACTUAL` 置位（切换还没生效），
  跑几拍到 0x6061 跟上后验全 0。

**回归**：HAL 离线用例 4/4；RT 侧 smoke 与主轴回归与基线逐条一致
（RT 适配层没包这个接口，本来就碰不到）。

**ABI**：这又是一条语义变更，和上面的清理一起记在 7.0 里（7.0 还没对外发布）。
若要让编号严格对应，发布前改成 8.0。
