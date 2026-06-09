#include "backlash_comp.h"
#include <cstring>
#include <gtest/gtest.h>

/* ========================================================================= */
/* 基础功能测试                                                              */
/* ========================================================================= */

TEST(TestBacklashComp, DisabledReturnsZero)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 0;
    par_cmp.bl_value = 0.010;

    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(TestBacklashComp, PositiveDirection)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.010; /* bl_rate >= bl_value → 瞬时完成 */

    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.010);
}

TEST(TestBacklashComp, NegativeDirection)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.010;

    double out = calculate_backlash_compensation(&par_cmp, &state, -1.0, 0);
    EXPECT_DOUBLE_EQ(out, -0.010);
}

/* ========================================================================= */
/* bl_enable=2 快移/切削切换测试                                              */
/* ========================================================================= */

TEST(TestBacklashComp, RapidModeUsesBlValue2)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 2;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_value2 = 0.020;
    par_cmp.bl_rate = 0.020; /* bl_rate >= bl_value2 → 瞬时 */

    /* is_rapid=1 应使用 bl_value2 */
    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 1);
    EXPECT_DOUBLE_EQ(out, 0.020);
}

TEST(TestBacklashComp, CuttingModeUsesBlValue)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 2;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_value2 = 0.020;
    par_cmp.bl_rate = 0.010;

    /* is_rapid=0 应使用 bl_value */
    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.010);
}

/* ========================================================================= */
/* 平滑步进 (bl_rate) 测试                                                   */
/* ========================================================================= */

TEST(TestBacklashComp, RampingGradualMultiCycle)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010; /* 目标 10μm */
    par_cmp.bl_rate = 0.004;  /* 每周期步进 4μm，需 3 周期完成 */

    /* 第1周期: 0 → 4μm */
    double out1 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_NEAR(out1, 0.004, 1e-9);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);

    /* 第2周期: 4μm → 8μm */
    double out2 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_NEAR(out2, 0.008, 1e-9);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);

    /* 第3周期: 8μm → 10μm (diff=2 < rate=4, snap to target) */
    double out3 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out3, 0.010);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
}

TEST(TestBacklashComp, RampingInstantWhenRateGEValue)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.015; /* bl_rate > bl_value → 1 周期完成 */

    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.010);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
}

TEST(TestBacklashComp, RampingInstantWhenRateZero)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.0; /* bl_rate == 0 → 1 周期完成 */

    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.010);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);
}

/* ========================================================================= */
/* 方向切换测试                                                              */
/* ========================================================================= */

TEST(TestBacklashComp, DirectionReversalTriggersRamping)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;

    /* 先正向运动到稳态 */
    par_cmp.bl_rate = 0.010;
    double out1 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out1, 0.010);
    EXPECT_EQ(state.phase, COMP_PHASE_ACTIVE);

    /* 换向: 应从 +0.010 向 -0.010 平滑过渡 */
    par_cmp.bl_rate = 0.004; /* 每周期 4μm */
    double out2 = calculate_backlash_compensation(&par_cmp, &state, -1.0, 0);
    /* +0.010 → 步进 0.004 向 -0.010: 第一周期 +0.006 */
    EXPECT_NEAR(out2, 0.006, 1e-9);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
}

/* ========================================================================= */
/* 首次运动封锁测试                                                          */
/* ========================================================================= */

TEST(TestBacklashComp, FirstMovementBlocksOutput)
{
    BacklashCompState state;
    backlash_comp_init(&state);

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.010;

    /* 未运动: 强制返回 0 */
    double out = calculate_backlash_compensation(&par_cmp, &state, 0.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.0);
    EXPECT_FALSE(state.has_moved);
}

TEST(TestBacklashComp, FirstMovementTriggersRamping)
{
    BacklashCompState state;
    backlash_comp_init(&state);

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.004;

    /* 静止时不输出 */
    double out1 = calculate_backlash_compensation(&par_cmp, &state, 0.0, 0);
    EXPECT_DOUBLE_EQ(out1, 0.0);

    /* 首次运动: 进入 RAMPING，开始步进 */
    double out2 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_TRUE(state.has_moved);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
    EXPECT_NEAR(out2, 0.004, 1e-9);
}

/* ========================================================================= */
/* 安全限幅测试                                                              */
/* ========================================================================= */

TEST(TestBacklashComp, SafetyHardLimit)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 2.0; /* 远大于 MAX_COMP_LIMIT_PER (1.0) */
    par_cmp.bl_rate = 2.0;

    double out = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, MAX_COMP_LIMIT_PER);
}

TEST(TestBacklashComp, SafetyHardLimitNegative)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 2.0;
    par_cmp.bl_rate = 2.0;

    double out = calculate_backlash_compensation(&par_cmp, &state, -1.0, 0);
    EXPECT_DOUBLE_EQ(out, -MAX_COMP_LIMIT_PER);
}

/* ========================================================================= */
/* 停止保持测试                                                              */
/* ========================================================================= */

TEST(TestBacklashComp, StopHoldsLastValue)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.010;

    /* 正向运动中 */
    double out1 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out1, 0.010);

    /* 停止: 应保持上周期输出 */
    double out2 = calculate_backlash_compensation(&par_cmp, &state, 0.0, 0);
    EXPECT_DOUBLE_EQ(out2, 0.010);
}

/* ========================================================================= */
/* 空指针安全测试                                                            */
/* ========================================================================= */

TEST(TestBacklashComp, NullParCmpReturnsZero)
{
    BacklashCompState state;
    backlash_comp_init(&state);

    double out = calculate_backlash_compensation(NULL, &state, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(TestBacklashComp, NullStateReturnsZero)
{
    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;

    double out = calculate_backlash_compensation(&par_cmp, NULL, 1.0, 0);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

/* ========================================================================= */
/* 使能切换测试                                                              */
/* ========================================================================= */

TEST(TestBacklashComp, EnableToggleTriggersRamping)
{
    BacklashCompState state;
    backlash_comp_init(&state);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.bl_enable = 1;
    par_cmp.bl_value = 0.010;
    par_cmp.bl_rate = 0.004;

    /* 先到稳态 */
    double out1 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    /* 0 → 4μm (ramping) */
    EXPECT_NEAR(out1, 0.004, 1e-9);

    /* 继续到稳态 */
    double out2 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    /* 4μm → 8μm */
    EXPECT_NEAR(out2, 0.008, 1e-9);

    /* 关闭使能: 目标变为 0，应进入 RAMPING 向 0 过渡 */
    par_cmp.bl_enable = 0;
    double out3 = calculate_backlash_compensation(&par_cmp, &state, 1.0, 0);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
    /* 上周期 0.008，diff = -0.008，步进 0.004，输出 0.004 */
    EXPECT_NEAR(out3, 0.004, 1e-9);
}
