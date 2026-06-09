#include "pitch_comp.h"
#include <cmath>
#include <cstring>
#include <gtest/gtest.h>

/* ========================================================================= */
/* 测试辅助：构建 3 点螺距补偿表                                            */
/*                                                                           */
/* 测量点: [0, 10, 20] (mm)                                                  */
/* 正向补偿: [1, 3, 5] (μm) → 转换后 [0.001, 0.003, 0.005] mm               */
/* 反向补偿: [-2, -4, -6] (μm) → 转换后 [-0.002, -0.004, -0.006] mm         */
/* 斜率 = 0.0002 mm/mm (正向) / -0.0002 mm/mm (反向)                          */
/* ========================================================================= */
static pitch_comp_table_t make_three_point_table()
{
    pitch_comp_table_t table;
    std::memset(&table, 0, sizeof(table));

    double pts[] = {0.0, 10.0, 20.0};
    int16_t fwd[] = {1, 3, 5};
    int16_t rev[] = {-2, -4, -6};
    int ret = generate_pitch_comp_table(&table, pts, fwd, rev, 3);
    EXPECT_EQ(ret, PITCH_TABLE_OK);
    return table;
}

/* ========================================================================= */
/* generate_pitch_comp_table() 测试                                          */
/* ========================================================================= */

TEST(TestPitchComp, TableInitSuccess)
{
    pitch_comp_table_t table = make_three_point_table();

    EXPECT_EQ(table.entries, 3);
    /* 边界点坐标 */
    EXPECT_DOUBLE_EQ(table.array[0].nominal, -__DBL_MAX__);
    EXPECT_DOUBLE_EQ(table.array[4].nominal, __DBL_MAX__);
    /* 实际测量点 */
    EXPECT_DOUBLE_EQ(table.array[1].nominal, 0.0);
    EXPECT_DOUBLE_EQ(table.array[2].nominal, 10.0);
    EXPECT_DOUBLE_EQ(table.array[3].nominal, 20.0);
    /* 游标初始化 */
    EXPECT_EQ(table.entry, &table.array[0]);
    /* 斜率预计算：中间段 (0→10) */
    EXPECT_FLOAT_EQ(table.array[1].fwd_slope, 0.0002f);
    EXPECT_FLOAT_EQ(table.array[1].rev_slope, -0.0002f);
}

TEST(TestPitchComp, TableNullTablePtr)
{
    double pts[] = {0.0};
    int16_t fwd[] = {0};
    int16_t rev[] = {0};
    int ret = generate_pitch_comp_table(NULL, pts, fwd, rev, 1);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NULL_PTR);
}

TEST(TestPitchComp, TableNullMeasPosArr)
{
    pitch_comp_table_t table;
    int16_t fwd[] = {0};
    int16_t rev[] = {0};
    int ret = generate_pitch_comp_table(&table, NULL, fwd, rev, 1);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NULL_PTR);
}

TEST(TestPitchComp, TableNullFwdCompArr)
{
    pitch_comp_table_t table;
    double pts[] = {0.0};
    int16_t rev[] = {0};
    int ret = generate_pitch_comp_table(&table, pts, NULL, rev, 1);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NULL_PTR);
}

TEST(TestPitchComp, TableNullRevCompArr)
{
    pitch_comp_table_t table;
    double pts[] = {0.0};
    int16_t fwd[] = {0};
    int ret = generate_pitch_comp_table(&table, pts, fwd, NULL, 1);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NULL_PTR);
}

TEST(TestPitchComp, TableZeroPoints)
{
    pitch_comp_table_t table;
    double pt = 0.0;
    int16_t fwd = 0;
    int16_t rev = 0;
    int ret = generate_pitch_comp_table(&table, &pt, &fwd, &rev, 0);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NUM_POINTS);
}

TEST(TestPitchComp, TableExceedMaxSize)
{
    pitch_comp_table_t table;
    /* 用一个大数组但传 num_points=257 触发上限检查 */
    double pts[257];
    int16_t fwd[257];
    int16_t rev[257];
    std::memset(pts, 0, sizeof(pts));
    std::memset(fwd, 0, sizeof(fwd));
    std::memset(rev, 0, sizeof(rev));
    int ret = generate_pitch_comp_table(&table, pts, fwd, rev, 257);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NUM_POINTS);
}

TEST(TestPitchComp, TableNotAscending)
{
    pitch_comp_table_t table;
    double pts[] = {0.0, 20.0, 10.0}; /* 第三点逆序 */
    int16_t fwd[] = {0, 0, 0};
    int16_t rev[] = {0, 0, 0};
    int ret = generate_pitch_comp_table(&table, pts, fwd, rev, 3);
    EXPECT_EQ(ret, PITCH_TABLE_ERR_NOT_ASCENDING);
}

/* ========================================================================= */
/* calculate_pitch_compensation() — 安全/边界测试                            */
/* ========================================================================= */

TEST(TestPitchComp, CalcNullState)
{
    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    pitch_comp_table_t table = make_three_point_table();
    double out = calculate_pitch_compensation(NULL, &par_cmp, &table, 5.0, 1.0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(TestPitchComp, CalcNullParCmp)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    pitch_comp_table_t table = make_three_point_table();
    double out = calculate_pitch_compensation(&state, NULL, &table, 5.0, 1.0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(TestPitchComp, CalcNullTable)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    double out = calculate_pitch_compensation(&state, &par_cmp, NULL, 5.0, 1.0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(TestPitchComp, CalcTypeDisabled)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 0; /* 禁能 */

    pitch_comp_table_t table = make_three_point_table();
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(TestPitchComp, CalcEmptyTable)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;

    /* 建一个空表 */
    pitch_comp_table_t table;
    std::memset(&table, 0, sizeof(table));
    table.entries = 0;
    table.entry = &table.array[0];

    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

/*
 * 注意：以下稳态测试中 table 使用 float 存误差值和斜率，插值结果有 float 级
 * 精度误差（~1e-7 量级），因此使用 EXPECT_NEAR(1e-6) 代替 EXPECT_DOUBLE_EQ。
 */

/* ========================================================================= */
/* calculate_pitch_compensation() — 单向补偿 (type=1)                         */
/* ========================================================================= */

TEST(TestPitchComp, CalcUniDirInterpolation)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 1; /* 避免 type≠last_type 触发意外的类型切换 */
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * pos=5 位于 [0, 10] 区间：
     * delta = 5 - 0 = 5
     * target = 0.001 + 0.0002×5 = 0.002
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out, 0.002, 1e-6);
}

TEST(TestPitchComp, CalcUniDirBelowRange)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 1;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /* pos=-5 < 首个测量点 0.0 → 游标停在边界哨兵点，持住首点补偿值 0.001 */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, -5.0, 1.0);
    EXPECT_NEAR(out, 0.001, 1e-6);
}

TEST(TestPitchComp, CalcUniDirAboveRange)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 1;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /* pos=25 > 末个测量点 20.0 → 游标停在末点，持住末点补偿值 0.005 */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 25.0, 1.0);
    EXPECT_NEAR(out, 0.005, 1e-6);
}

/* ========================================================================= */
/* calculate_pitch_compensation() — 双向补偿 (type=2)                         */
/* ========================================================================= */

TEST(TestPitchComp, CalcBiDirFwdVelocity)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 2; /* 避免意外的类型切换 */
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 2;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /* pos=5, vel>0: 应使用正向表 (fwd) */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out, 0.002, 1e-6);
}

TEST(TestPitchComp, CalcBiDirRevVelocity)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 2;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 2;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * pos=15, vel<0: 应使用反向表 (rev)
     * 位于 [10, 20] 区间，delta = 15-10 = 5
     * rev_comp = -0.004, rev_slope = -0.0002
     * target = -0.004 + (-0.0002)×5 = -0.005
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 15.0, -1.0);
    EXPECT_NEAR(out, -0.005, 1e-6);
}

TEST(TestPitchComp, CalcBiDirStationaryHolds)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 2;
    state.phase = COMP_PHASE_ACTIVE;
    state.current_output = 0.002; /* 模拟上一周期的输出 */

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 2;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * vel=0 (< VEL_THRESHOLD), type=2:
     * 应保持上一周期输出不变
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 0.0);
    EXPECT_DOUBLE_EQ(out, 0.002);
}

/* ========================================================================= */
/* calculate_pitch_compensation() — 状态机测试                                */
/* ========================================================================= */

TEST(TestPitchComp, CalcPowerOnVelZeroStaysIdle)
{
    /* 模拟上电后首个伺服周期：补偿使能已开启，但轴尚未运动 (vel=0) */
    PitchCompState state;
    pitch_comp_init(&state, 0.005);

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /* 连续多个周期 vel=0，均应死守 0 输出，状态机停留在 IDLE */
    for (int i = 0; i < 3; i++)
    {
        double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 0.0);
        EXPECT_DOUBLE_EQ(out, 0.0);
        EXPECT_FALSE(state.has_moved);
        EXPECT_EQ(state.phase, COMP_PHASE_IDLE);
    }
}

TEST(TestPitchComp, CalcFirstMovementBlocks)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /* 未运动且 vel=0: 强制返回 0 */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 0.0);
    EXPECT_DOUBLE_EQ(out, 0.0);
    EXPECT_FALSE(state.has_moved);
}

TEST(TestPitchComp, CalcFirstMovementEnters)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * 首次运动 (vel > VEL_THRESHOLD):
     *   1. has_moved=false → 首次运动检测 → has_moved=true, RAMPING
     *   2. 注：type(1) ≠ last_type(0) 也触发切换，但已在 RAMPING 中，无影响
     *   3. step_limit=0.005, target≈0.002 → diff < step → 直接到位 → ACTIVE
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_TRUE(state.has_moved);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
    EXPECT_NEAR(out, 0.002, 1e-6);
}

TEST(TestPitchComp, CalcRampingToActive)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.003); /* step_limit = 3μm */

    state.has_moved = true;
    state.last_type = 1; /* 避免意外的类型切换 */
    state.phase = COMP_PHASE_RAMPING;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * pos=5, target≈0.002, step=0.003
     * diff < step → 直接到位 → ACTIVE
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out, 0.002, 1e-6);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
}

TEST(TestPitchComp, CalcRampingMultiStep)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.0005); /* step_limit = 0.5μm */

    state.has_moved = true;
    state.last_type = 1; /* 避免意外的类型切换 */
    state.phase = COMP_PHASE_RAMPING;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * pos=5, target≈0.002, step=0.0005
     * 第 1 周期: 0 → 0.0005,  RAMPING
     * 第 2 周期: 0.0005 → 0.001, RAMPING
     * 第 3 周期: 0.001 → 0.0015, RAMPING
     * 第 4 周期: 0.0015 → target (diff≤step), ACTIVE
     */
    double out1 = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out1, 0.0005, 1e-9);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);

    double out2 = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out2, 0.001, 1e-9);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);

    double out3 = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out3, 0.0015, 1e-9);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);

    double out4 = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out4, 0.002, 1e-6);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
}

TEST(TestPitchComp, CalcTypeSwitchToZero)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.003);

    /* 模拟之前 type=1 时已进入 ACTIVE，输出稳定在 0.002 */
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;
    state.last_type = 1;
    state.current_output = 0.002;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 0; /* 切换到禁能 */
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * 新逻辑：静止时冻结输出，type 变化仅在运动时检测。
     * 周期1 (静止): 冻结输出，phase / has_moved 均不变。
     */
    double out1 = calculate_pitch_compensation(&state, &par_cmp, &table, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(out1, 0.002);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
    EXPECT_TRUE(state.has_moved);

    /*
     * 周期2 (运动): 检测到 type 1→0，触发 RAMPING。
     * target=0, diff=-0.002, |diff| < step(0.003) → 直接到位 → IDLE
     */
    double out2 = calculate_pitch_compensation(&state, &par_cmp, &table, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(out2, 0.0);
    EXPECT_EQ(state.phase, COMP_PHASE_IDLE);
    EXPECT_FALSE(state.has_moved);
}

TEST(TestPitchComp, CalcTypeSwitchRamping)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.0005); /* 小步进限制，确保切换后保持在 RAMPING */

    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;
    state.last_type = 1;
    state.current_output = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 2; /* 1 → 2 切换 */
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * type 从 1 切换为 2: 触发 RAMPING 重入场
     * step=0.0005, target≈0.002, diff≈0.002 > step → 步进一步，仍为 RAMPING
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
    EXPECT_NEAR(out, 0.0005, 1e-9);
}

/* ========================================================================= */
/* calculate_pitch_compensation() — 其他测试                                  */
/* ========================================================================= */

TEST(TestPitchComp, CalcSafetyHardLimit)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 1;
    state.phase = COMP_PHASE_ACTIVE;
    state.current_output = MAX_COMP_LIMIT_PER; /* 预设输出=1.0，避免 target_mismatch 误触发 RAMPING */

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 2000.0; /* 放大到远超 MAX_COMP_LIMIT_PER */

    /* 构建单点表，fwd=1μm→0.001mm，×2000 = 2.0 > 1.0 */
    pitch_comp_table_t table;
    std::memset(&table, 0, sizeof(table));
    double pts[] = {0.0};
    int16_t fwd[] = {1};
    int16_t rev[] = {1};
    ASSERT_EQ(generate_pitch_comp_table(&table, pts, fwd, rev, 1), PITCH_TABLE_OK);

    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(out, MAX_COMP_LIMIT_PER); /* 钳位至 1.0 */
}

TEST(TestPitchComp, CalcSafetyHardLimitNegative)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 2;
    state.phase = COMP_PHASE_ACTIVE;
    state.current_output = -MAX_COMP_LIMIT_PER; /* 预设输出=-1.0，避免 target_mismatch 误触发 RAMPING */

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 2;
    par_cmp.pitch_factor = 2000.0;

    /* 单点表，rev=-1μm→-0.001mm，×2000 = -2.0 < -1.0 */
    pitch_comp_table_t table;
    std::memset(&table, 0, sizeof(table));
    double pts[] = {0.0};
    int16_t fwd[] = {0};
    int16_t rev[] = {-1};
    ASSERT_EQ(generate_pitch_comp_table(&table, pts, fwd, rev, 1), PITCH_TABLE_OK);

    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 0.0, -1.0);
    EXPECT_DOUBLE_EQ(out, -MAX_COMP_LIMIT_PER); /* 钳位至 -1.0 */
}

TEST(TestPitchComp, CalcPitchFactor)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 1;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 0.5;

    pitch_comp_table_t table = make_three_point_table();

    /*
     * pos=5, 插值≈0.002, ×0.5 = 0.001
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NEAR(out, 0.001, 1e-6);
}

TEST(TestPitchComp, CalcCursorLazyInit)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005);
    state.has_moved = true;
    state.last_type = 1;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;
    par_cmp.pitch_factor = 1.0;

    pitch_comp_table_t table = make_three_point_table();
    table.entry = NULL; /* 模拟游标未初始化（如建表失败后手动修复） */

    /*
     * 游标为 NULL 时应自动初始化为 &array[0]，随后正常插值
     */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 5.0, 1.0);
    EXPECT_NE(table.entry, (pitch_comp_entry_t *) NULL);
    EXPECT_NEAR(out, 0.002, 1e-6);
}

/* ========================================================================= */
/* 保留原有测试                                                              */
/* ========================================================================= */

TEST(TestPitchComp, RampingStepLimit)
{
    PitchCompState state;
    pitch_comp_init(&state, 0.005); /* 设置 5um 的步进限制 */

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.pitch_type = 1;     /* 单向补偿 */
    par_cmp.pitch_factor = 1.0; /* 倍率 1.0 */

    /* 构建一个简单的单点补偿表 */
    pitch_comp_table_t table;
    std::memset(&table, 0, sizeof(table));

    double pts[] = {0.0};
    int16_t fwd[] = {10}; /* 10μm = 0.01mm */
    int16_t rev[] = {10};
    int ret = generate_pitch_comp_table(&table, pts, fwd, rev, 1);
    ASSERT_EQ(ret, 0);

    /* 模拟首次运动触发入场 (vel > VEL_THRESHOLD) */
    double out = calculate_pitch_compensation(&state, &par_cmp, &table, 0.0, 1.0);

    /* 首次输出不应超过 step_limit */
    EXPECT_LE(std::abs(out), 0.005);
}
