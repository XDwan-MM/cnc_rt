#include "comp_common.h"
#include "perp_comp.h"
#include <cmath>
#include <cstring>
#include <gtest/gtest.h>

/**
 * @brief 测试基础计算逻辑
 * 验证公式：Target = (RefPos - RefPoint) * (Angle * 1e-6)
 */
TEST(TestPerpComp, BasicCalculation)
{
    PerpCompState state;
    perp_comp_init(&state, 0.01); // 步进限制设大一点以便直接观察结果
    state.has_moved = true;       // 模拟已启动
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));

    // 设置参数：Y轴(idx 1)补偿，基准轴为X(idx 0)
    par_cmp.squ1_enable = 1;
    par_cmp.squ1_input_ax = 0; // X轴
    par_cmp.squ1_refn = 100.0; // 参考点在 100mm
    par_cmp.squ1_ang = 50.0;   // 角度 50um/m

    double axis_pos[3] = {200.0, 0.0, 0.0}; // 当前X在200mm

    // 理想补偿 = (200 - 100) * (50 * 1e-6) = 100 * 0.00005 = 0.005mm
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.1);

    EXPECT_DOUBLE_EQ(out, 0.005);
}

/**
 * @brief 测试双项叠加逻辑
 * 两个补偿项的影响应线性相加
 */
TEST(TestPerpComp, DoubleItemOverlay)
{
    PerpCompState state;
    perp_comp_init(&state, 0.1);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));

    // 项1: X对Y的影响
    par_cmp.squ1_enable = 1;
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0; // 0.1um/mm

    // 项2: Z对Y的影响
    par_cmp.squ2_enable = 1;
    par_cmp.squ2_input_ax = 2; // Z轴
    par_cmp.squ2_ang = 200.0;  // 0.2um/mm

    double axis_pos[3] = {10.0, 0.0, 10.0}; // X=10, Z=10

    // 总补偿 = (10*0.0001) + (10*0.0002) = 0.001 + 0.002 = 0.003mm
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.1);

    EXPECT_NEAR(out, 0.003, 1e-9);
}

/**
 * @brief 测试安全硬限幅
 * 无论计算值多大，输出不得超过 MAX_COMP_LIMIT_PER (1.0mm)
 */
TEST(TestPerpComp, SafetyHardLimit)
{
    PerpCompState state;
    perp_comp_init(&state, 0.1);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;
    state.last_perp1_enabled = true; /* 匹配 squ1_enable=1，避免触发 status_changed 回退到 RAMPING */

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 1;
    par_cmp.squ1_ang = 5000.0; // 设置一个极大的角度，使 ideal 值远超 1.0

    double axis_pos[3] = {1000.0, 0.0, 0.0};

    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.1);

    // 结果必须被截断到 1.0
    EXPECT_DOUBLE_EQ(out, MAX_COMP_LIMIT_PER);
}

/**
 * @brief 测试自补偿屏蔽
 * 如果基准轴等于当前补偿轴，补偿应为 0（防止逻辑死循环）
 */
TEST(TestPerpComp, SelfCompensationMasking)
{
    PerpCompState state;
    perp_comp_init(&state, 0.1);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 1;
    par_cmp.squ1_input_ax = 1; // 基准轴也是Y轴
    par_cmp.squ1_ang = 100.0;

    double axis_pos[3] = {0.0, 500.0, 0.0};
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.1);

    EXPECT_DOUBLE_EQ(out, 0.0);
}

/**
 * @brief 测试静止时使能变化进入 WAIT_MOTION
 * ACTIVE 状态下使能从 1→0，三轴均静止，应进入 WAIT_MOTION 而非立即斜坡
 */
TEST(TestPerpComp, EnableChangeWhileStationaryEntersWaitMotion)
{
    PerpCompState state;
    perp_comp_init(&state, 0.001);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;
    state.last_perp1_enabled = true;
    state.current_output = 0.005;
    /* 初始化位置快照，与当前轴位置一致，模拟静止状态 */
    state.last_axis_pos[0] = 100.0;
    state.last_axis_pos[1] = 0.0;
    state.last_axis_pos[2] = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    /* 设置为已使能状态，然后关闭 */
    par_cmp.squ1_enable = 0; // 1→0
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0;

    double axis_pos[3] = {100.0, 0.0, 0.0}; // 所有轴与快照一致，静止
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.0);

    EXPECT_EQ(state.phase, COMP_PHASE_WAIT_MOTION);
    EXPECT_DOUBLE_EQ(out, 0.005); // 输出冻结，不步进
}

/**
 * @brief 测试 WAIT_MOTION 下连续多周期静止保持输出不变
 */
TEST(TestPerpComp, WaitMotionHoldsOutputWhileStationary)
{
    PerpCompState state;
    perp_comp_init(&state, 0.001);
    state.has_moved = true;
    state.phase = COMP_PHASE_WAIT_MOTION;
    state.last_perp1_enabled = false; // 使能已为 0，状态已更新
    state.last_perp2_enabled = false;
    state.current_output = 0.005;
    state.last_axis_pos[0] = 100.0;
    state.last_axis_pos[1] = 0.0;
    state.last_axis_pos[2] = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 1; // 使能开启了，但之前变化时静止，在等运动
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0;

    double axis_pos[3] = {100.0, 0.0, 0.0};

    // 连续 3 个周期静止，输出应始终保持不变
    for (int cyc = 0; cyc < 3; cyc++)
    {
        double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.0);
        EXPECT_EQ(state.phase, COMP_PHASE_WAIT_MOTION);
        EXPECT_DOUBLE_EQ(out, 0.005);
    }
}

/**
 * @brief 测试 WAIT_MOTION 下补偿轴运动触发转换
 * 补偿轴 vel_cmd > VEL_THRESHOLD → 转到 RAMPING，本周期输出冻结
 */
TEST(TestPerpComp, WaitMotionTransitionsWhenCompAxisMoves)
{
    PerpCompState state;
    perp_comp_init(&state, 0.001);
    state.has_moved = true;
    state.phase = COMP_PHASE_WAIT_MOTION;
    state.last_perp1_enabled = true; // 状态已同步（上一周期使能变化时已更新）
    state.current_output = 0.005;
    state.last_axis_pos[0] = 100.0;
    state.last_axis_pos[1] = 0.0;
    state.last_axis_pos[2] = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 1; // 使能已开启，在等运动
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0;
    par_cmp.squ1_refn = 100.0; // 参考点 100，当前位置也是 100，target=0

    double axis_pos[3] = {100.0, 0.0, 0.0};

    // 本周期：补偿轴运动 (vel_cmd=1.0)
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 1.0);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
    EXPECT_DOUBLE_EQ(out, 0.005); // 本周期冻结

    // 下周期：RAMPING 开始步进，target=0 (squ1_enable=1, ref_pos=100, refn=100)
    // target = (100-100)*100*1e-6 = 0，diff = -0.005，步进 0.001 → 0.004
    double out2 = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 1.0);
    EXPECT_NEAR(out2, 0.004, 1e-9);
}

/**
 * @brief 测试 WAIT_MOTION 下非补偿轴运动触发转换
 * 补偿轴静止 (vel_cmd=0)，但其他轴位置变化 → 转到 RAMPING
 */
TEST(TestPerpComp, WaitMotionTransitionsWhenOtherAxisMoves)
{
    PerpCompState state;
    perp_comp_init(&state, 0.001);
    state.has_moved = true;
    state.phase = COMP_PHASE_WAIT_MOTION;
    state.last_perp1_enabled = true; // 状态已同步（使能已开启，在等运动）
    state.current_output = 0.005;
    state.last_axis_pos[0] = 100.0; // 快照 X=100.0
    state.last_axis_pos[1] = 0.0;
    state.last_axis_pos[2] = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 1; // 使能开启中
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0;
    par_cmp.squ1_refn = 100.0; // target = (100.001-100)*100*1e-6 = 0.0001um ≈ 0

    // X 轴从 100.0 移动到 100.001，delta=0.001 >> POS_DELTA_THRESHOLD
    // 补偿轴 (Y) vel_cmd=0，静止
    double axis_pos[3] = {100.001, 0.0, 0.0};
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.0);

    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
    EXPECT_DOUBLE_EQ(out, 0.005); // 本周期冻结
}

/**
 * @brief 测试 WAIT_MOTION 中再次翻转使能，仍保持 WAIT_MOTION
 * 连续两次使能变化都在静止状态，应保持等待，输出不变
 */
TEST(TestPerpComp, WaitMotionReEnterOnSecondToggle)
{
    PerpCompState state;
    perp_comp_init(&state, 0.001);
    state.has_moved = true;
    state.phase = COMP_PHASE_WAIT_MOTION;
    state.last_perp1_enabled = false; // 当前使能=0
    state.current_output = 0.005;
    state.last_axis_pos[0] = 100.0;
    state.last_axis_pos[1] = 0.0;
    state.last_axis_pos[2] = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 1; // 开启使能 (0→1)
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0;

    double axis_pos[3] = {100.0, 0.0, 0.0};

    // 第一次翻转：0→1，静止
    double out1 = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.0);
    EXPECT_EQ(state.phase, COMP_PHASE_WAIT_MOTION);
    EXPECT_DOUBLE_EQ(out1, 0.005);

    // 第二次翻转：1→0，仍静止
    par_cmp.squ1_enable = 0;
    double out2 = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 0.0);
    EXPECT_EQ(state.phase, COMP_PHASE_WAIT_MOTION);
    EXPECT_DOUBLE_EQ(out2, 0.005);
}

/**
 * @brief 回归测试：运动中使能变化直接进入 RAMPING
 * 应保持与旧行为一致，不进入 WAIT_MOTION
 */
TEST(TestPerpComp, EnableChangeWhileMovingGoesDirectlyToRamping)
{
    PerpCompState state;
    perp_comp_init(&state, 0.001);
    state.has_moved = true;
    state.phase = COMP_PHASE_ACTIVE;
    state.last_perp1_enabled = true;
    state.current_output = 0.005;
    state.last_axis_pos[0] = 100.0;
    state.last_axis_pos[1] = 0.0;
    state.last_axis_pos[2] = 0.0;

    CompParams par_cmp;
    std::memset(&par_cmp, 0, sizeof(par_cmp));
    par_cmp.squ1_enable = 0; // 1→0
    par_cmp.squ1_input_ax = 0;
    par_cmp.squ1_ang = 100.0;
    par_cmp.squ1_refn = 100.0;

    double axis_pos[3] = {100.0, 0.0, 0.0};

    // 补偿轴运动中 (vel_cmd=1.0)，应直接 RAMPING
    double out = calculate_perpendicularity_total(&par_cmp, &state, 1, axis_pos, 1.0);
    EXPECT_EQ(state.phase, COMP_PHASE_RAMPING);
    // target=0，current_output=0.005，step=0.001 → 0.004
    EXPECT_NEAR(out, 0.004, 1e-9);
}
