#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "global.h"
#include "mpg.h"
#include "mpg_planner.h"
#include "test_env.h"

namespace
{

uint32_t g_hw_count = 0;
uint32_t g_hw_axis_and_rate = 0;
int g_hw_count_result = 0;
int g_hw_axis_and_rate_result = 0;
bool g_hw_count_write_on_failure = false;
bool g_hw_axis_and_rate_write_on_failure = false;
int g_hw_count_calls = 0;
int g_hw_axis_and_rate_calls = 0;

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
        rt_data->nc_event = SYS_EVENT_NONE;
        rt_data->ncu_data.cycle = 0.001;
        rt_data->axis_data[AXIS_X].max_feed_acc = 100.0;
        rt_data->axis_data[AXIS_X].mpg_spd = 10.0;
        rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 0.0;
        g_hw_count_result = 0;
        g_hw_axis_and_rate_result = 0;
        g_hw_count_write_on_failure = false;
        g_hw_axis_and_rate_write_on_failure = false;
        g_hw_count_calls = 0;
        g_hw_axis_and_rate_calls = 0;
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
    g_hw_count_calls++;
    if (value != nullptr && (g_hw_count_result == 0 || g_hw_count_write_on_failure))
        *value = g_hw_count;
    return g_hw_count_result;
}

extern "C" int HW_Get_Axis_And_Rate(uint32_t *axis_and_rate)
{
    g_hw_axis_and_rate_calls++;
    if (axis_and_rate != nullptr &&
        (g_hw_axis_and_rate_result == 0 || g_hw_axis_and_rate_write_on_failure))
        *axis_and_rate = g_hw_axis_and_rate;
    return g_hw_axis_and_rate_result;
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

TEST_F(TestMpg, TurningAxisSelectionOffDeceleratesThePreviouslyActiveAxis)
{
    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(10);

    const double velocity_before_off = rt_data->axis_data[AXIS_X].cmd_vel;
    ASSERT_GT(velocity_before_off, 0.0);

    configure_mpg_input(4000, HANDWHEEL_AXIS_OFF, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_LT(rt_data->axis_data[AXIS_X].cmd_vel, velocity_before_off);

    run_mpg_cycles(200);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestMpg, HardwareReadFailuresStopAndDiscardCountsUntilRecovery)
{
    struct FailureCase
    {
        int count_result;
        int switch_result;
        bool write_on_failure;
    };
    const FailureCase cases[] = {
        {-1, 0, false}, {1, 0, true},
        {0, -1, false}, {0, 1, true},
        {-1, 1, false}, {1, -1, true},
    };

    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;
    for (const FailureCase &failure : cases)
    {
        SCOPED_TRACE(::testing::Message() << "count=" << failure.count_result
                                         << " switch=" << failure.switch_result
                                         << " writes=" << failure.write_on_failure);
        handwheel_handle_estop();
        rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 0.0;
        rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur = 0.0;
        g_hw_count_result = 0;
        g_hw_axis_and_rate_result = 0;
        configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        run_mpg_cycles(20);
        const double velocity_before_failure = rt_data->axis_data[AXIS_X].cmd_vel;
        ASSERT_GT(velocity_before_failure, 0.1);

        g_hw_count_result = failure.count_result;
        g_hw_axis_and_rate_result = failure.switch_result;
        g_hw_count_write_on_failure = failure.write_on_failure;
        g_hw_axis_and_rate_write_on_failure = failure.write_on_failure;
        configure_mpg_input(1000000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X100);
        const int count_calls_before = g_hw_count_calls;
        const int switch_calls_before = g_hw_axis_and_rate_calls;
        handle_mpg_cmd();
        EXPECT_EQ(g_hw_count_calls, count_calls_before + 1);
        EXPECT_EQ(g_hw_axis_and_rate_calls, switch_calls_before + 1);
        EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, velocity_before_failure - 0.1, 1.0e-9);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);

        run_mpg_cycles(200);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
        const double stopped_position = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
        g_hw_count_result = 0;
        g_hw_axis_and_rate_result = 0;
        configure_mpg_input(2000000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        run_mpg_cycles(5);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, stopped_position);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

        configure_mpg_input(2000004, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, stopped_position);
    }
}

TEST_F(TestMpg, InvalidAxisAndMultiplierCodesStopAndDiscardCounts)
{
    struct InvalidSelection
    {
        int axis;
        int multiplier;
    };
    const InvalidSelection selections[] = {
        {HANDWHEEL_AXIS_OFF, HANDWHEEL_RATE_X1},
        {HANDWHEEL_AXIS_X | HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1},
        {HANDWHEEL_AXIS_X, HANDWHEEL_RATE_OFF},
        {HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1 | HANDWHEEL_RATE_X10},
    };

    for (const InvalidSelection &selection : selections)
    {
        SCOPED_TRACE(::testing::Message() << "axis=" << selection.axis
                                         << " multiplier=" << selection.multiplier);
        handwheel_handle_estop();
        rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 0.0;
        configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        run_mpg_cycles(20);
        const double velocity_before_failure = rt_data->axis_data[AXIS_X].cmd_vel;
        ASSERT_GT(velocity_before_failure, 0.1);

        configure_mpg_input(8000, selection.axis, selection.multiplier);
        handle_mpg_cmd();
        EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, velocity_before_failure - 0.1, 1.0e-9);
        run_mpg_cycles(200);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
        const double stopped_position = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
        configure_mpg_input(12000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        run_mpg_cycles(5);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, stopped_position);
        configure_mpg_input(12004, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, stopped_position);
    }
}

TEST_F(TestMpg, HardwareReadRecoveryPreservesVelocityAndResumesOnNewCounts)
{
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    const double velocity_before_failure = rt_data->axis_data[AXIS_X].cmd_vel;
    ASSERT_GT(velocity_before_failure, 0.3);

    g_hw_count_result = -1;
    configure_mpg_input(10000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, velocity_before_failure - 0.1, 1.0e-9);

    g_hw_count_result = 0;
    configure_mpg_input(20000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, velocity_before_failure - 0.2, 1.0e-9);
    const double velocity_after_recovery = rt_data->axis_data[AXIS_X].cmd_vel;
    configure_mpg_input(24000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, velocity_after_recovery + 0.1, 1.0e-9);
}

TEST_F(TestMpg, HardwareReadRecoveryOnAnotherAxisKeepsPreviousAxisDecelerating)
{
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);

    g_hw_axis_and_rate_result = -1;
    configure_mpg_input(8000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    const double velocity_after_failure = rt_data->axis_data[AXIS_X].cmd_vel;
    ASSERT_GT(velocity_after_failure, 0.2);

    g_hw_axis_and_rate_result = 0;
    configure_mpg_input(12000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, velocity_after_failure - 0.1, 1.0e-9);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur, 0.0);

    configure_mpg_input(16000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);
    EXPECT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    run_mpg_cycles(200);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestMpg, AlternatingReadFailuresDoNotReplayCounts)
{
    handle_mpg_cmd();
    for (uint32_t cycle = 1; cycle <= 10; cycle++)
    {
        g_hw_count_result = -1;
        configure_mpg_input(cycle * 1000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        g_hw_count_result = 0;
        configure_mpg_input(cycle * 1000 + 500, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
        handle_mpg_cmd();
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);
        EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    }
    configure_mpg_input(10504, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);
}

TEST_F(TestMpg, ReadFailureRecoveryDiscardsCountsAcrossCounterWrap)
{
    configure_mpg_input(UINT32_MAX - 100, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    g_hw_count_result = -1;
    configure_mpg_input(UINT32_MAX - 20, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    g_hw_count_result = 0;
    configure_mpg_input(UINT32_MAX - 1, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

    configure_mpg_input(2, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);
    EXPECT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

    handwheel_handle_estop();
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 0.0;
    configure_mpg_input(UINT32_MAX - 100, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    g_hw_count_result = -1;
    configure_mpg_input(2, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    g_hw_count_result = 0;
    configure_mpg_input(100, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(5);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    configure_mpg_input(104, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 0.0);
}

TEST_F(TestMpg, HardwareFailureStopsAllAxesBeforeMachineLockTakesEffect)
{
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    configure_mpg_input(4000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(8000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(5);
    const double x_velocity_before_failure = rt_data->axis_data[AXIS_X].cmd_vel;
    const double y_velocity_before_failure = rt_data->axis_data[AXIS_Y].cmd_vel;
    ASSERT_GT(x_velocity_before_failure, 0.1);
    ASSERT_GT(y_velocity_before_failure, 0.1);

    g_hw_count_result = -1;
    rt_data->nc_event = SYS_EVENT_MAC_LOCK_ON;
    handle_mpg_cmd();
    rt_data->nc_event = SYS_EVENT_NONE;
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, x_velocity_before_failure - 0.1, 1.0e-9);
    EXPECT_NEAR(rt_data->axis_data[AXIS_Y].cmd_vel, y_velocity_before_failure - 0.1, 1.0e-9);
    EXPECT_FALSE(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]);

    for (int cycle = 0; cycle < 200 && !rt_data->sys_state[SYS_IS_MAC_LOCK_READY]; cycle++)
    {
        handle_mpg_cmd();
        if (rt_data->axis_data[AXIS_X].cmd_vel > 1.0e-9 ||
            rt_data->axis_data[AXIS_Y].cmd_vel > 1.0e-9)
        {
            EXPECT_FALSE(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]);
        }
    }
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);
    EXPECT_TRUE(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]);
}

TEST_F(TestMpg, SwitchingAxesContinuesDeceleratingThePreviousAxis)
{
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;

    configure_mpg_input(0, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(10);

    const double velocity_before_switch = rt_data->axis_data[AXIS_X].cmd_vel;
    ASSERT_GT(velocity_before_switch, 0.0);

    configure_mpg_input(4000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_LT(rt_data->axis_data[AXIS_X].cmd_vel, velocity_before_switch);

    configure_mpg_input(8000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);

    run_mpg_cycles(200);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestMpg, SwitchingBackToAnAxisPreservesItsDecelerationVelocity)
{
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;

    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    configure_mpg_input(4000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();

    const double previous_velocity = rt_data->axis_data[AXIS_X].cmd_vel;
    ASSERT_GT(previous_velocity, 0.1);
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, previous_velocity - 0.1, 1.0e-9);
}

TEST_F(TestMpg, ChangingMultiplierOrReenablingAxisKeepsVelocityContinuous)
{
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);

    double previous_velocity = rt_data->axis_data[AXIS_X].cmd_vel;
    ASSERT_GT(previous_velocity, 0.3);
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X10);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, previous_velocity - 0.1, 1.0e-9);

    configure_mpg_input(4000, HANDWHEEL_AXIS_OFF, HANDWHEEL_RATE_X10);
    handle_mpg_cmd();
    previous_velocity = rt_data->axis_data[AXIS_X].cmd_vel;
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X10);
    handle_mpg_cmd();
    EXPECT_NEAR(rt_data->axis_data[AXIS_X].cmd_vel, previous_velocity - 0.1, 1.0e-9);
}

TEST_F(TestMpg, MachineLockWaitsForAllPreviouslySelectedAxesToStop)
{
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;
    rt_data->axis_data[AXIS_Z].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Z].mpg_spd = 10.0;

    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    configure_mpg_input(4000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    configure_mpg_input(8000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(5);
    configure_mpg_input(8000, HANDWHEEL_AXIS_Z, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();

    rt_data->nc_event = SYS_EVENT_MAC_LOCK_ON;
    handle_mpg_cmd();
    rt_data->nc_event = SYS_EVENT_NONE;
    ASSERT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    ASSERT_GT(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);
    EXPECT_FALSE(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]);

    for (int cycle = 0; cycle < 200 && !rt_data->sys_state[SYS_IS_MAC_LOCK_READY]; cycle++)
    {
        handle_mpg_cmd();
        if (rt_data->axis_data[AXIS_X].cmd_vel > 1.0e-9 ||
            rt_data->axis_data[AXIS_Y].cmd_vel > 1.0e-9)
        {
            EXPECT_FALSE(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]);
        }
    }
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_Y].cmd_vel, 0.0);
    EXPECT_TRUE(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]);
}

TEST_F(TestMpg, PositionResynchronizationDiscardsPreviousAxisStopState)
{
    rt_data->axis_data[AXIS_Y].max_feed_acc = 100.0;
    rt_data->axis_data[AXIS_Y].mpg_spd = 10.0;

    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    configure_mpg_input(4000, HANDWHEEL_AXIS_Y, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    ASSERT_GT(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

    const double externally_set_position = 12.0;
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = externally_set_position;
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, externally_set_position);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, externally_set_position);
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

TEST_F(TestMpg, InvalidPlannerLimitsDiscardPendingMotion)
{
    mpg_planner_t planner = {};

    planner.axis = AXIS_X;
    planner.current_position = 3.0;
    planner.reference_position = 3.0;
    planner.target_position = 8.0;
    planner.current_velocity = 2.0;
    planner.max_velocity = 10.0;
    planner.max_acceleration = 0.0;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_DOUBLE_EQ(planner.current_velocity, 0.0);
    EXPECT_DOUBLE_EQ(planner.target_position, 3.0);

    planner.max_acceleration = 100.0;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_DOUBLE_EQ(planner.current_position, 3.0);
    EXPECT_DOUBLE_EQ(planner.current_velocity, 0.0);

    planner.target_position = 8.0;
    planner.current_velocity = 2.0;
    mpg_planner_step(&planner, std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(planner.current_velocity, 0.0);
    EXPECT_DOUBLE_EQ(planner.target_position, 3.0);
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_DOUBLE_EQ(planner.current_position, 3.0);
}

TEST_F(TestMpg, PlannerRecoversNonFinitePositionOnlyFromKnownCommandOrReference)
{
    mpg_planner_t planner = {};
    const double invalid_position = std::numeric_limits<double>::quiet_NaN();

    planner.axis = AXIS_X;
    planner.max_acceleration = 100.0;
    planner.max_velocity = 10.0;
    planner.current_position = invalid_position;
    planner.reference_position = 4.0;
    planner.target_position = 8.0;
    planner.current_velocity = 2.0;
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 5.0;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_DOUBLE_EQ(planner.current_position, 5.0);
    EXPECT_DOUBLE_EQ(planner.target_position, 5.0);
    EXPECT_DOUBLE_EQ(planner.current_velocity, 0.0);

    planner.current_position = invalid_position;
    planner.reference_position = 4.0;
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = invalid_position;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_DOUBLE_EQ(planner.current_position, 4.0);
    EXPECT_DOUBLE_EQ(planner.target_position, 4.0);

    planner.current_position = invalid_position;
    planner.reference_position = invalid_position;
    planner.target_position = 8.0;
    planner.current_velocity = 2.0;
    mpg_planner_step(&planner, rt_data->ncu_data.cycle);
    EXPECT_TRUE(std::isnan(planner.current_position));
    EXPECT_DOUBLE_EQ(planner.target_position, 8.0);
    EXPECT_DOUBLE_EQ(planner.current_velocity, 0.0);
}

TEST_F(TestMpg, NonFiniteCommandPositionRecoversWithoutACommandedJump)
{
    handle_mpg_cmd();
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    run_mpg_cycles(20);
    const double previous_position = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;

    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = std::numeric_limits<double>::quiet_NaN();
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, previous_position);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_pre, previous_position);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestMpg, NonFiniteInitialPositionWaitsForAValidCommandPosition)
{
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = std::numeric_limits<double>::quiet_NaN();
    handle_mpg_cmd();
    EXPECT_TRUE(std::isnan(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur));
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = 7.0;
    configure_mpg_input(4000, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 7.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);

    configure_mpg_input(4004, HANDWHEEL_AXIS_X, HANDWHEEL_RATE_X1);
    handle_mpg_cmd();
    EXPECT_GT(rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, 7.0);
}

TEST_F(TestMpg, LimitDecelerationIsMarkedDoneOnlyAfterVelocityStops)
{
    mpg_planner_t planner = {};

    planner.axis = AXIS_X;
    planner.max_acceleration = 100.0;
    planner.max_velocity = 10.0;
    planner.current_velocity = 0.5;
    planner.active = true;
    rt_data->axis_data[AXIS_X].status.is_fwd_limit = true;

    for (int cycle = 0; cycle < 10 && planner.active; cycle++)
    {
        planner.target_position = planner.current_position;
        mpg_planner_step(&planner, rt_data->ncu_data.cycle);
        if (planner.active)
        {
            EXPECT_FALSE(rt_data->axis_data[AXIS_X].status.is_decel_done);
        }
    }
    EXPECT_FALSE(planner.active);
    EXPECT_TRUE(rt_data->axis_data[AXIS_X].status.is_decel_done);
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
