# 螺距补偿单元测试补充计划

## Context

`tests/test_pitch_comp.cpp` 目前只有 1 个测试用例（`RampingStepLimit`），远未覆盖螺距补偿模块的完整行为。需要对照 `src/pitch_comp.c` 和 `src/pitch_comp_table_init.c` 补充系统性的单元测试。

## 涉及文件

- **修改**: `tests/test_pitch_comp.cpp` — 新增 ~25 个测试用例
- 其余文件不变

## 测试用例清单

### generate_pitch_comp_table() — 建表函数

| # | 测试名 | 场景 | 期望 |
|---|--------|------|------|
| 1 | TableInitSuccess | 3 点合法输入，建表成功 | PITCH_TABLE_OK，验证 entries=3、边界点坐标、斜率 |
| 2 | TableNullTablePtr | table=NULL | PITCH_TABLE_ERR_NULL_PTR |
| 3 | TableNullMeasPosArr | meas_pos_arr_mm=NULL | PITCH_TABLE_ERR_NULL_PTR |
| 4 | TableNullFwdCompArr | fwd_comp_arr_um=NULL | PITCH_TABLE_ERR_NULL_PTR |
| 5 | TableNullRevCompArr | rev_comp_arr_um=NULL | PITCH_TABLE_ERR_NULL_PTR |
| 6 | TableZeroPoints | num_points=0 | PITCH_TABLE_ERR_NUM_POINTS |
| 7 | TableExceedMaxSize | num_points=257 (>256) | PITCH_TABLE_ERR_NUM_POINTS |
| 8 | TableNotAscending | 测量点非升序 [0, 20, 10] | PITCH_TABLE_ERR_NOT_ASCENDING |

### calculate_pitch_compensation() — 补偿计算

#### 安全/边界

| # | 测试名 | 场景 | 期望 |
|---|--------|------|------|
| 9 | CalcNullState | state=NULL | 返回 0 |
| 10 | CalcNullParCmp | par_cmp=NULL | 返回 0 |
| 11 | CalcNullTable | table=NULL | 返回 0 |
| 12 | CalcTypeDisabled | pitch_type=0 且已运动 | 返回 0 |
| 13 | CalcEmptyTable | entries=0, type=1 | 返回 0 |

#### 单向补偿 (type=1)

| # | 测试名 | 场景 | 期望 |
|---|--------|------|------|
| 14 | CalcUniDirInterpolation | 3 点表，pos 在中间区段 | 线性插值结果正确 |
| 15 | CalcUniDirBelowRange | pos < 第一个测量点 | 持住首点补偿值（边界哨兵 slope=0） |
| 16 | CalcUniDirAboveRange | pos > 最后一个测量点 | 持住末点补偿值（边界哨兵 slope=0） |

#### 双向补偿 (type=2)

| # | 测试名 | 场景 | 期望 |
|---|--------|------|------|
| 17 | CalcBiDirFwdVelocity | vel>0, type=2 | 使用正向表 (fwd_comp/slope) |
| 18 | CalcBiDirRevVelocity | vel<0, type=2 | 使用反向表 (rev_comp/slope) |
| 19 | CalcBiDirStationaryHolds | vel=0, type=2, 先运动到 ACTIVE | 静止时保持上一周期输出 |

#### 状态机

| # | 测试名 | 场景 | 期望 |
|---|--------|------|------|
| 20 | CalcPowerOnVelZeroStaysIdle | 上电后 vel=0 多周期 | 输出 0，has_moved=false，phase=IDLE |
| 21 | CalcFirstMovementBlocks | has_moved=false, vel=0 | 输出 0 |
| 22 | CalcFirstMovementEnters | has_moved=false, vel>0 | 进入 RAMPING，首步 ≤ step_limit |
| 22 | CalcRampingToActive | 从 RAMPING 经多周期到 ACTIVE | 逐步到位后切换 ACTIVE |
| 23 | CalcTypeSwitchToZero | type=1→0 切换 | 进入 RAMPING，平滑退出到 IDLE，has_moved 重置 |
| 24 | CalcTypeSwitchRamping | type=1→2 切换 | 触发 RAMPING 重入场 |

#### 其他

| # | 测试名 | 场景 | 期望 |
|---|--------|------|------|
| 25 | CalcSafetyHardLimit | 大补偿值 + factor | 钳位至 ±MAX_COMP_LIMIT_PER (1.0) |
| 26 | CalcPitchFactor | pitch_factor=0.5 | 输出 = 插值结果 × 0.5 |
| 27 | CalcCursorLazyInit | entry=NULL 的 table | 自动初始化游标，正常计算 |

## 测试辅助模式

- 使用辅助函数 `make_table()` 构建测试用表，返回 `pitch_comp_table_t`
- 对稳态测试（直接验证 ACTIVE 输出），手动设置 `state.has_moved=true`, `state.phase=COMP_PHASE_ACTIVE`
- 对状态机测试，使用 `pitch_comp_init()` 初始化，多次调用驱动状态迁移

## 验证

```bash
rm -rf build && ./build.sh debug test
```

编译零警告，所有已有 (6 pitch + 17 backlash) + 新增 (~25 pitch) 测试通过。
