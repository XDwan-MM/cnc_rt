#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "global.h"
#include "mpg.h"
#include "mpg_planner.h"
#include "test_env.h"

namespace
{

uint32_t g_hw_count = 0;
uint32_t g_hw_axis_and_rate = 0;

void configure_mpg_input(uint32_t count, int axis_switch, int rate_switch)
{
    g_hw_count = count;
    g_hw_axis_and_rate = (static_cast<uint32_t>(axis_switch) << 3) |
                         static_cast<uint32_t>(rate_switch);
}

void run_mpg_cycles(int cycles)
{
    for (int i = 0; i < cycles; i++)
        handle_mpg_cmd();
}

class TestMpg : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady())
        {
            GTEST_SKIP() << testEnvMessage();
        }

        handwheel_handle_estop();
        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        std::memset(rt_data->chan_data, 0, sizeof(rt_data->chan_data));
        rt_data->ncu_data.cycle = 0.001;
        rt_data->axis_data[AXIS_X].max_feed_acc = 100.0;
        rt_data->axis_data[AXIS_X].mpg_spd = 10.0;
        rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 0.0;
        configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    }

    void TearDown() override
    {
        handwheel_handle_estop();
    }
};

} // namespace

extern "C" int HW_Get_Count(uint32_t *value)
{
    if (value != nullptr)
        *value = g_hw_count;
    return 0;
}

extern "C" int HW_Get_Axis_And_Rate(uint32_t *axis_and_rate)
{
    if (axis_and_rate != nullptr)
        *axis_and_rate = g_hw_axis_and_rate;
    return 0;
}

TEST_F(TestMpg, SwitchMappingsKeepSupportedAxesAndRejectUnsupportedAxes)
{
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

    handwheel_handle_estop();
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X10);
    handle_mpg_cmd();
    configure_mpg_input(400, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X10);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);

    handwheel_handle_estop();
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 0.0;
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X100);
    handle_mpg_cmd();
    configure_mpg_input(400, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X100);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);

    handwheel_handle_estop();
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    configure_mpg_input(0, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur, 0.0);
}

TEST_F(TestMpg, InvalidAxisSelectionClearsChannelOutput)
{
    rt_data->chan_data[0].cmd_vel = 3.0;
    configure_mpg_input(0, HANDWHEEL_AXIS_OFF, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->chan_data[0].cmd_vel, 0.0);

    rt_data->chan_data[0].cmd_vel = 3.0;
    configure_mpg_input(0, HANDWHEEL_AXIS_4, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->chan_data[0].cmd_vel, 0.0);

    rt_data->chan_data[0].cmd_vel = 3.0;
    configure_mpg_input(0, HANDWHEEL_AXIS_5, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->chan_data[0].cmd_vel, 0.0);
}

TEST_F(TestMpg, PlannerDiscardsFirstCountAndHandlesCounterWrap)
{
    mpg_planner_t planner = {};

    planner.axis = AXIS_X;
    planner.first_cycle = true;
    planner.count = 10U;
    planner.previous_count = 0U;
    planner.multiplier = 1U;
    planner.max_velocity = 10.0;
    planner.max_acceleration = 100.0;
    mpg_planner_update_target(&planner);
    EXPECT_DOUBLE_EQ(planner.target_position, 0.0);

    planner.first_cycle = false;
    planner.count = 0U;
    planner.previous_count = UINT32_MAX;
    mpg_planner_update_target(&planner);
    EXPECT_GT(planner.target_position, 0.0);
}

TEST_F(TestMpg, AxisOutputPreservesPreviousCommandPosition)
{
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();

    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    const double first_position = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    EXPECT_GT(first_position, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_pre, 0.0);

    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_pre, first_position);
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, first_position);
}

TEST_F(TestMpg, ChannelVelocityIsFeedAxisVectorMagnitude)
{
    rt_data->axis_data[AXIS_Y].cmd_vel = 3.0;
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();

    const double x_velocity = rt_data->axis_data[AXIS_X].cmd_vel;
    EXPECT_GT(x_velocity, 0.0);
    EXPECT_NEAR(rt_data->chan_data[0].cmd_vel,
                std::sqrt(x_velocity * x_velocity + 9.0), 1.0e-12);
}

TEST_F(TestMpg, PlannerMultiplierScalesCountDistance)
{
    mpg_planner_t planner = {};

    planner.axis = AXIS_X;
    planner.count = 400U;
    planner.previous_count = 0U;
    planner.multiplier = 1U;
    /* 让停止距离大于本测试的最大目标距离，单独验证倍率换算。 */
    planner.max_velocity = 100.0;
    planner.max_acceleration = 100.0;
    planner.reference_position = 0.0;
    mpg_planner_update_target(&planner);
    EXPECT_DOUBLE_EQ(planner.target_position, 0.1);

    planner.previous_count = 0U;
    planner.multiplier = 10U;
    planner.target_position = 0.0;
    mpg_planner_update_target(&planner);
    EXPECT_DOUBLE_EQ(planner.target_position, 1.0);

    planner.previous_count = 0U;
    planner.multiplier = 100U;
    planner.target_position = 0.0;
    mpg_planner_update_target(&planner);
    EXPECT_DOUBLE_EQ(planner.target_position, 10.0);
}

TEST_F(TestMpg, PlannerAppliesAccelerationAndLimitProtection)
{
    mpg_planner_t planner = {};

    planner.axis = AXIS_X;
    planner.max_acceleration = 100.0;
    planner.max_velocity = 10.0;
    planner.target_position = 1.0;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_GT(planner.current_velocity, 0.0);
    EXPECT_LE(planner.current_velocity, planner.max_velocity);

    planner.max_acceleration = 0.0;
    planner.current_velocity = 5.0;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_DOUBLE_EQ(planner.current_velocity, 0.0);
    EXPECT_FALSE(planner.active);
}

TEST_F(TestMpg, PositiveAndNegativeLimitsRejectMatchingCountDirection)
{
    mpg_planner_t planner = {};

    planner.axis = AXIS_X;
    planner.first_cycle = false;
    planner.previous_count = 10U;
    planner.count = 20U;
    planner.multiplier = 1U;
    planner.max_acceleration = 100.0;
    planner.max_velocity = 10.0;
    planner.target_position = 0.0;
    planner.reference_position = 0.0;
    rt_data->axis_data[AXIS_X].status.is_fwd_limit = true;
    mpg_planner_update_target(&planner);
    EXPECT_DOUBLE_EQ(planner.target_position, 0.0);

    rt_data->axis_data[AXIS_X].status.is_fwd_limit = false;
    rt_data->axis_data[AXIS_X].status.is_rev_limit = true;
    planner.previous_count = 20U;
    planner.count = 10U;
    mpg_planner_update_target(&planner);
    EXPECT_DOUBLE_EQ(planner.target_position, 0.0);
}

TEST_F(TestMpg, ExternalDecelerationUsesMinimumEnabledPositiveLimit)
{
    rt_data->chan_data[0].ex_dec_enable = true;
    rt_data->sys_state[SYS_IS_MPG_EX_DEC_ENABLE] = true;
    rt_data->chan_data[0].ex_dec2_enable = true;
    rt_data->chan_data[0].ex_dec3_enable = true;
    rt_data->chan_data[0].ex_dec4_enable = true;
    rt_data->chan_data[0].ex_dec5_enable = true;
    rt_data->axis_data[AXIS_X].mpg_ex_dec1_vel = 4.0;
    rt_data->axis_data[AXIS_X].mpg_ex_dec2_vel = 3.0;
    rt_data->axis_data[AXIS_X].mpg_ex_dec3_vel = 1.0;
    rt_data->axis_data[AXIS_X].mpg_ex_dec4_vel = 2.0;
    rt_data->axis_data[AXIS_X].mpg_ex_dec5_vel = 5.0;
    for (int function = 0; function < SYS_EXTERNAL_DECEL_FUNCTION_COUNT; function++)
        rt_data->axis_data[AXIS_X].status.external_decel_active[function][0] = true;

    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);

    EXPECT_LE(rt_data->axis_data[AXIS_X].cmd_vel, 1.0 + 1.0e-9);
    EXPECT_DOUBLE_EQ(rt_data->chan_data[0].cmd_vel, rt_data->axis_data[AXIS_X].cmd_vel);
}

TEST_F(TestMpg, ExternalDecelerationIgnoresOppositeDirectionAndRestoresBaseSpeed)
{
    rt_data->chan_data[0].ex_dec_enable = true;
    rt_data->sys_state[SYS_IS_MPG_EX_DEC_ENABLE] = true;
    rt_data->chan_data[0].ex_dec2_enable = true;
    rt_data->axis_data[AXIS_X].mpg_ex_dec1_vel = 1.0;
    rt_data->axis_data[AXIS_X].mpg_ex_dec2_vel = 0.25;
    rt_data->axis_data[AXIS_X].status.external_decel_active[0][0] = true;
    rt_data->axis_data[AXIS_X].status.external_decel_active[1][1] = true;

    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    EXPECT_LE(rt_data->axis_data[AXIS_X].cmd_vel, 1.0 + 1.0e-9);

    handwheel_handle_estop();
    std::memset(rt_data->axis_data[AXIS_X].status.external_decel_active, 0,
                sizeof(rt_data->axis_data[AXIS_X].status.external_decel_active));
    rt_data->axis_data[AXIS_X].status.external_decel_active[1][1] = true;
    configure_mpg_input(1000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    EXPECT_LE(rt_data->axis_data[AXIS_X].cmd_vel, 0.25 + 1.0e-9);

    std::memset(rt_data->axis_data[AXIS_X].status.external_decel_active, 0,
                sizeof(rt_data->axis_data[AXIS_X].status.external_decel_active));
    run_mpg_cycles(10);
    EXPECT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.25);
}

TEST_F(TestMpg, ExternalDecelerationGlobalSwitchDisablesAllFunctions)
{
    rt_data->chan_data[0].ex_dec_enable = false;
    rt_data->axis_data[AXIS_X].mpg_ex_dec1_vel = 0.0;
    rt_data->axis_data[AXIS_X].status.external_decel_active[0][0] = true;

    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);

    EXPECT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestMpg, MpgExternalDecelerationStateDisablesAllFunctions)
{
    rt_data->chan_data[0].ex_dec_enable = true;
    rt_data->sys_state[SYS_IS_MPG_EX_DEC_ENABLE] = false;
    rt_data->axis_data[AXIS_X].mpg_ex_dec1_vel = 0.0;
    rt_data->axis_data[AXIS_X].status.external_decel_active[0][0] = true;

    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);

    EXPECT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestMpg, EstopClearsPlannerAndVelocityOutputs)
{
    rt_data->axis_data[AXIS_X].cmd_vel = 3.0;
    rt_data->chan_data[0].cmd_vel = 3.0;
    handwheel_handle_estop();

    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->chan_data[0].cmd_vel, 0.0);
}

TEST_F(TestMpg, ExternalAxisPositionChangeResynchronizesPlannerForXyz)
{
    const int axes[] = {AXIS_X, AXIS_Y, AXIS_Z};
    const int switches[] = {HANDWHEEL_AXIS_X, HANDWHEEL_AXIS_Y, HANDWHEEL_AXIS_Z};

    for (int index = 0; index < 3; index++)
    {
        handwheel_handle_estop();
        for (int axis = AXIS_X; axis <= AXIS_Z; axis++)
        {
            rt_data->axis_data[axis].max_feed_acc = 100.0;
            rt_data->axis_data[axis].mpg_spd = 10.0;
            rt_data->axis_data[axis].wcs_cmd_pos_cur = 10.0 * (axis + 1);
        }

        const int axis = axes[index];
        configure_mpg_input(100U, switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        configure_mpg_input(4100U, switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        run_mpg_cycles(20);

        const double external_position = rt_data->axis_data[axis].wcs_cmd_pos_cur + 5.0;
        rt_data->axis_data[axis].wcs_cmd_pos_cur = external_position;
        configure_mpg_input(12100U, switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();

        EXPECT_DOUBLE_EQ(rt_data->axis_data[axis].wcs_cmd_pos_cur, external_position);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[axis].cmd_vel, 0.0);

        configure_mpg_input(12104U, switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        EXPECT_GT(rt_data->axis_data[axis].wcs_cmd_pos_cur, external_position);
    }
}

TEST_F(TestMpg, CrossAxisExternalPositionChangeDoesNotRestoreStaleMpgAxis)
{
    const int mpg_axes[] = {AXIS_X, AXIS_Y, AXIS_Z};
    const int mpg_switches[] = {HANDWHEEL_AXIS_X, HANDWHEEL_AXIS_Y, HANDWHEEL_AXIS_Z};
    const int external_axes[] = {AXIS_Y, AXIS_Z, AXIS_X};

    for (int index = 0; index < 3; index++)
    {
        handwheel_handle_estop();
        for (int axis = AXIS_X; axis <= AXIS_Z; axis++)
        {
            rt_data->axis_data[axis].max_feed_acc = 100.0;
            rt_data->axis_data[axis].mpg_spd = 10.0;
            rt_data->axis_data[axis].wcs_cmd_pos_cur = 10.0 * (axis + 1);
        }

        configure_mpg_input(200U, mpg_switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        configure_mpg_input(4200U, mpg_switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        run_mpg_cycles(20);

        const double mpg_position = rt_data->axis_data[mpg_axes[index]].wcs_cmd_pos_cur;
        const int external_axis = external_axes[index];
        const double external_position = rt_data->axis_data[external_axis].wcs_cmd_pos_cur + 7.0;
        rt_data->axis_data[external_axis].wcs_cmd_pos_cur = external_position;
        configure_mpg_input(13000U, mpg_switches[index], HANDWHEEL_RATE_X1);
        handle_mpg_cmd();

        EXPECT_DOUBLE_EQ(rt_data->axis_data[mpg_axes[index]].wcs_cmd_pos_cur, mpg_position);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[external_axis].wcs_cmd_pos_cur, external_position);
    }
}
