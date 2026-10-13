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
| `/home/mxh/CNC/port_work` | **移植工作副本**（cnc_rt_last 的拷贝 + 分支 `new-hal`） | 在这里干活 |
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

### D. 移植之后又做的几批（2026-10-10 ~ 10-12，详情见 HAL 的 `docs/续接说明.md` D–I 段）

| 批 | 做了什么 | 影响面 |
|---|---|---|
| D | 函数改名 + **ABI 闸门改成真编译期 `#error`**；两套回归加界面闸门 | port_work |
| E | 删掉两个"现场确认"开关：台账只保留"参数域给全了没有" | port_work |
| F | **主轴定向：发出控制字后先等 10 ms 再采完成位**（手册要求）→ `HalCAxisCfg.vendor_orient_delay_ms`，ABI **7.0→7.1** | HAL + port_work |
| G | **速度单位可配**（默认 rpm；pulse/s 时 HAL 双向换算）+ **进给轴显示换算成 mm/min** + 按需取用接口 `rt_axis_view_*`，ABI **7.1→7.2** | HAL + port_work |
| H | **刚性攻丝闭环**：M29 起点设零（新增 `rt_hal_axis_set_pos`）+ 插补的绝对角度由 `emcmot` 每拍下发（以前算了不发现、且 READY 阶段把主轴锁死），见第十节 | port_work |
| I | **IO 映像段长上限收成一处 + 启动期断言**（防越界写 / 防无声失效）| port_work |
| 其他 | 适配层把"轴尚未使能"这类预期内失败从错误日志摘出去；`pm_munit` 由 `int` 改 `double`（亚毫米截断）；`build.sh` 失败返回非 0 | port_work |

让 HAL 那边的知识（手册结论、ABI 变更理由、未决项）也成了可交接的文档：
`HAL/docs/续接说明.md`（详细）、`HAL/docs/上机检查单.md`（现场照着划的）、
`HAL/docs/仿真进HAL方案.md`（下一阶段）。


---

## 三、移植现状（`port_work` / 分支 `new-hal`）

### 当前状态（每次交接前更新这一块）

| | |
|---|---|
| HAL HEAD | `4d1183f 文档：IO 映像段长断言；上机清单补两条（计数模式、主轴当量单位）` |
| port_work HEAD | `c9f2e21 …`（IO 段长断言那批；**本文件自己那次提交不在内**）|
| ABI | **7.2**（`HAL_C_ABI_MAJOR` 7 / `HAL_C_ABI_MINOR` 2）—— 布局变更，库 / 头 / RT / 固件包**必须一起换** |
| `bin/cnc_rt` | sha256 前 16 位 `e69d065c47f036aa` |
| `bin/cnc_rt_d` | sha256 前 16 位 `384fbc92111993a1` |
| 离线回归板 | HAL 用例 4/4；离线配置自检 45/45；轴量接口自检 22/22；冒烟 5 场景 + 主轴全 PASS |

> 改了 HAL 的头/源码之后：`bash build_all.sh`（在 HAL）→ `lib/cnc_hal/sync.sh`（在 port_work）
> → `./build.sh`。**RT 侧的 ABI 闸门是编译期 `#error`**，忘了改期望值就编不过（这是故意的）。

### 已提交（最近）

```
c9f2e21 IO 映像段长：上限收成一处，并在启动期断言（防越界写 / 防无声失效）
7f4d232 刚性攻丝闭环：角度算了就要下发，并对齐零点
976215c 按需取用接口：指令/反馈脉冲 + 两种速度口径（不发共享内存）
a41e10b 蓝图：进给轴显示换算成 mm/min；速度单位值接通；加 rpm 口径的读接口
10d1883 蓝图：定向等 10ms；ABI 闸门期望 7.1
8826b33 适配层：把"轴尚未使能"这类预期内拒绝从错误日志里摘出去
7097af6 适配层：主轴模式握手只在"发过请求且已请求使能"时做（去掉假警报）
d5d6ae0 删掉两个"现场确认"开关：台账只留"参数域给全了没有"
010d866 rt_hal_config：函数改名 + ABI 闸门改成真编译期；两套回归加界面闸门
1dd319b rt_hal_config.h：按 rt_hal_adapter.h 的规格补全函数注释
af30d6e 接入层与移植层收口：复位链、使能单一写者、主轴上限、驱动器故障码
b97ea13 接入层注释补全；vendored 同步到 HAL 30fec81
d6c4dbf vendored 同步：hal_config_api.h 补注释
6bc2a41 接入新hal
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
| `lib/cnc_hal` vendored 副本 | 完成（现在 ABI **7.2**，由 `lib/cnc_hal/sync.sh` 同步）|
| **编译（debug + release）** | 完成，都出二进制 |

### 台账：只反映"参数域给全了没有"

`src/hal/rt_hal_config.c` 里曾经有两个现场确认开关（`RT_HAL_IO_MAP_CONFIRMED` /
`RT_HAL_MOTION_RANGE_APPROVED`），**已按拍板删掉**（2026-10-10）：逐点核对点表、
批准运动范围与量程是电气/工艺的职责，不该由移植代码用"不能动"去卡流程。

现在台账只剩"参数域给全了没有"（按轴算的 SCALING / ENC_OFF / ENCODER）：
**参数域为空时仍然会挡住运动**，四轴参数写全后自动清空。

> 当初移植时逐文件比对过 `port_work/src/hal/*` 与 `cnc_rt/src/hal/*`：没有任何 port_work
> 专属适配，所以整体覆盖是安全的。**现在 HAL 是唯一的源**：改完 HAL 跑
> `lib/cnc_hal/sync.sh` 把公共头/内部头/静态库/设备字典同步到 vendored 副本，
> 提交时把 `lib/cnc_hal` 一起带上（`SYNCED.txt` 记来源提交）。

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

## 四、验证结果（当前）

### 回归板（每次改完都跑这四条）

```
# 1) HAL 离线用例（含 ABI、DS402、定向、速度单位、readiness）
cd ../HAL && bash build_all.sh && ctest --test-dir build/all

# 2) 机床 profile 的离线校验（不进总线，只验配置能不能过 HAL 校验）
bash test/build_hal_config_check.sh          # 45 项

# 3) 轴量按需接口的纯换算自检
bash test/build_axis_view_check.sh           # 22 项

# 4) RT 侧冒烟 + 主轴（要先把界面和 RT 都停掉：共享内存只允许 2 个进程）
sudo ./test/rt_offline_smoke.sh 8 params params     # 场景：params/drverr/drverrf/estopmid/fault
sudo ./test/rt_offline_spindle.sh
```

| 项 | 结果 |
|---|---|
| HAL 用例 | 4/4（`hal_hardware_selftest` / `hal_public_link` / `hal_static_link` / `hal_regressions`）|
| 离线配置自检 | 45/45 |
| 轴量接口自检 | 22/22（脉冲/速度换算、拒绝路径、主轴角度）|
| 冒烟 5 场景 | 全 PASS（params 5 / drverr 9 / drverrf 9 / estopmid 8 / fault 11）|
| 主轴回归 | 全 PASS（含 M3 到速 / M5 停稳不带力矩 / M19 定向与重复 / 释放）|

### 几条"先红后绿"的证据（改哪条就复现哪条）

- **定向等待期**：把 `orient_delay_left` 短路 → `hal_regressions` 失败；恢复后 4/4。
- **IO 段长断言**：上限临时改 `10u`（仿真段长 X=26/Y=18）→ RT **拒绝启动**并打出超限原因；恢复后全过。
- **ABI 闸门**：期望值故意改错 → `build.sh` 返回非 0 且打印 `#error`（以前它返回 0，见第五节）。

### 移植期的历史结论（保留备查）

移植完成那轮跑过 `test/test.pro` 的 gtest：**79 用例、26 失败**，且
`cnc_rt`（我们的代码 + HAL）与 `port_work`（同事代码 + HAL）**失败清单逐条一致**
（`TestAuto` 12 / `TestJog` 10 / `TestCompData` 3 / `TestServoSample` 1），
说明是**测试本身过时**，不是移植带来的。两个进程在 tear-down 后都段错误，也是既有问题。

### 仍未做

- **NC 程序驱动 + 攻丝的离线回归**：离线 harness 不做程序译码，跑不了 M29/G33，
  所以攻丝只能上机验（判据见 `HAL/docs/上机检查单.md` §5.6）。
- **上机验证**：整个清单在 `HAL/docs/上机检查单.md`。

---

## 五、操作上的坑

| 坑 | 事实 | 正确做法 |
|---|---|---|
| ~~`build.sh` 失败也返回 rc=0~~ | **已修**（2026-10-10）：任一档失败就返回非 0 | 仍然建议看 `[OK]`/`[FAIL]` 与产物时间戳 |
| **`test/build.sh` 失败也返回 rc=0** | 同上 | 同上 |
| **qmake 不重编测试** | 换过 gtest 后旧 `.o` 仍注册到旧 runner，`Running 25 tests` 是假象 | **改完构建配置先 `./build.sh clean`** |
| **部署 ≠ 生效** | 已运行的进程仍用旧二进制 | **部署后必须重启 RT 进程** |
| **`strings` 匹配不了中文** | 返回 0 是假警报 | 用 `grep -c "串" 二进制` |
| **make 有时不重链** | `.o` 新但二进制旧 | 删掉 `bin/cnc_rt*` 再 make |
| **台账只剩"参数给全了没有"** | 两个现场确认开关（IO_MAP / MOTION）已删（2026-10-10）；参数域为空时运动仍被挡是**正常的** | 离线回归把四轴参数写全（`rt_offline_helper` 已经会写），台账自动清空 |
| **离线跑主轴要先补三个参数** | `PAR_AX_FEED_ACC`=0 → 加速度 0 → M3 转速永远不动；软限位=0 而位置是 -150 → 行程报警 → `SYS_IS_ALARMING` 每拍把主轴事件改写成 STOP；主轴倍率默认 0 → 目标速度乘成 0 | 见 `rt_offline_helper.c` 的 `set_axis()` / `write_spindle_params()` / `spl_set_override()` |
| **ABI 现在是 7.2，是破坏性变更** | 7.0→7.1 加 `vendor_orient_delay_ms`；7.1→7.2 加 `speed_unit` / `command_pulses_per_rev` / `actual_speed_rpm` | `struct_size` 变了 → HAL 库 + RT + 固件包**必须一起更新**；RT 侧有编译期 `#error` 闸门会拦 |
| **改了 HAL 头必须先 sync 再编 RT** | RT 的 make 不跟踪 `lib/cnc_hal/include` 的头文件依赖 | 改 HAL → `sync.sh` → `./build.sh`；同步后闸门会替你确认 ABI |
| **共享内存只允许 2 个进程** | 界面 + RT 各占一个；跑离线回归前两者都要停 | `rt_offline_smoke.sh` / `rt_offline_spindle.sh` 开头已自动清理并检查 |

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

按"卡在谁"分。**HAL 的 `docs/续接说明.md` 是详细版**（含 F/G/H/I 各批的来龙去脉），
`HAL/docs/上机检查单.md` 是现场照着划的版本。

### 台架上要做（P0）

| 项 | 说明 |
|---|---|
| 四个未知量 | 当量/方向、`act_vel` 口径、**M19 定向真值**（控制字已由手册确认）、**`0x603F` 清零口径** |
| 驱动器侧参数 | 清单 §0b：`C2-00=0`（速度单位，出厂默认是 pulse/s）、`P0-04/P0-05`、`b0-01`、`PA-00` 等 |
| **位置计数模式** | 清单 §1.7：面板 `F0-17` 转两圈，判断单圈模计数还是多圈 → 决定 `feedback_wrap` |
| **主轴当量那一栏** | 清单 §2.5：填 `360000`（千分之一度），填 360 会小 1000 倍 |
| 刚性攻丝 | 清单 §5.6：主轴真的跟着 Z 转 / 退回反向 / 螺纹规；看日志 target_pos 是否跟着 Z 变 |
| IO、报警、参数逐项 | 清单 §2–§9 |

### 代码（离线能验，按危险程度）

| 项 | 说明 |
|---|---|
| **模式切换失败没有 RT 侧超时动作** | 缺陷 #6：5000 拍 → 报错 + 重试 + 下使能。现在只有 HAL 侧超时 |
| **报警去抖 + `delAlarm`** | `alarm.c` 报了没有"清" |
| **当量只在 create 时快照** | 运行中改参数不生效（基线是每拍读）|
| **多段攻丝之间主轴会反向一整圈** | 见第十节 ②（本次只修了①③）|
| `enc_cntr` / 主轴位置那格 | 按需接口都有了，接不接界面待定 |
| `test/build.sh` 失败返回 0 | 与 `build.sh` 同样的坑，`build.sh` 已修，测试脚本没查 |

### 工程 / 跨产品

| 项 | 说明 |
|---|---|
| **HAL 推 gitea** | 破坏性 ABI（现在 7.2），推之前跟组里打招呼；也是"子模块/仿真进 HAL"的前提 |
| **仿真进 HAL** | 方案见 `HAL/docs/仿真进HAL方案.md`，四阶段 |
| HAL 变子模块 / `sync.sh` 去掉 mtime 闸门 | 二选一 |
| 字典里加蓝海华腾 | `devices.json` 手动加一条，`profile: "ds402"` |
| 报警号 120-131 的文本 | 随包 `alarmcontent.xml` 里没有条目 → 找 HMI 包负责人 |
| 驱动器故障要不要 `setAlarm` | 等 `0x603F` 口径 |
| 复位超时契约要不要改 | 见 `HAL/docs/续接说明.md` 未决表 |
| 速度单位要不要做成机床参数 | 现在在 profile（改要重编）；进 cncshm 是跨产品 |
| 单圈刻度接口（`hal_rt_spindle_write_pos`）| 现在**无调用者**（攻丝走绝对角度）→ 删或留给"手动转刻度" |
| 主轴零点归谁 | 目前只有攻丝起点在用 `set_pos`；回零流程（`ref.c`）不设零，要确认现场惯例 |

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

## 十、攻丝（G33.1 / G33.2）不同步 —— **①③ 已修，② 仍在**

查于 2026-10-09，修于 2026-10-12（方案 1：软件给绝对角度、主轴在 CSP 跟随）。

### ① 主轴角度压根没下发到驱动器 —— **已修**

- 原来：`interpolate_tap()`（`interpolation.c:159`）把角度写进
  `rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur`，但 `output_to_hal()` 只发 X/Y/Z，主轴没发。
- 现在：`output_to_hal()` 里**当前块是 `RIGID_TAP` 时**，每拍把 `wcs_cmd_pos_cur`
  （绝对累计角）发给主轴。只在攻丝段发，避免盖掉 M29 保位。

### ② 每个攻丝段开头会**回退一整圈** —— **仍在，待办**

`interpolate_tap()` 算的是**本段相对角度**（段起点 = 0）。而 `mac_cmd_pos` 是按拍差分
累加 `(wcs_cmd_pos_cur - wcs_cmd_pos_pre)` 进机床坐标的 → 上一段结束是 360、新段头一拍写 0
→ delta = −360 → 主轴倒转一圈。

现在下发的是 `wcs_cmd_pos_cur` 本身（不再走那条差分链），所以**单段攻丝的同步是对的**；
但**多段之间**仍会出现"从 360 回到 0"的整圈反向。段间本来没咬在工件里，危害有限，
但白转一圈。要修就得让角度**跨段累计**：要么插补侧维护一个全局基准（M29 时清零），
要么 `spl_control` 在换段时把已走的角度补进去。

### ③ M29 保位和这条累加链**不同源** —— **已修**

原来 M29 保位用 `s_spl_target_pos`（定向释放那一刻的反馈值），攻丝的角度是另一条链，
两者不等 → 起步会跳。现在 M29 进 READY 时统一为：

```
把"现在这个角度"定为 0°（rt_hal_axis_set_pos(SP, 0)）→ 写 0° 保持
→ 插补的角度就是从这个零点起算的绝对角（段起点 = 0，正好对齐）
```

`READY` 阶段**不再自己发位置**（以前每拍重发固定角度，等于把主轴锁死），
只把插补的指令角镜像给到位判据和日志。

### 验不了的部分

真实同步精度、驱动器跟不跟得上 —— **只能上机**（判据见《上机检查单》§5.6）。
离线 harness 不做程序译码，跑不了 M29/G33。

### 附：`spl_pos_planner.c` 是死代码

注释写着"单轴周期同步位置规划器"，全树**没人调用**。攻丝不需要它
（角度是从进给距离直接算的，不是独立规划出来的）。

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

**ABI**：这又是一条语义变更，和上面的清理一起记在 7.0 里。之后 7.0→7.1（加定向等待字段）、
7.1→7.2（加速度单位/换算字段与 `actual_speed_rpm`）都是**布局变更**。
**HAL 至今没对外发布**（没推 gitea，见第七节）——所以真要严格对应编号，
可以在发布前把 7.x 这几版整理成一次 8.0。
