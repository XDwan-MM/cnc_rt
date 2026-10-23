#include <gtest/gtest.h>
#include "spindle_internal.h"
#include "test_env.h"

namespace
{
class SpindleZeroSpeedAuthorizationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(testEnvReady()) << testEnvMessage();
        spindle_init();
    }
};
}

TEST_F(SpindleZeroSpeedAuthorizationTest, M3S0RetainsForwardStartRequest)
{
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW, 0.0));

    EXPECT_EQ(g_spindle.command_state, SPINDLE_CW);
    EXPECT_DOUBLE_EQ(g_spindle.command_speed, 0.0);
    EXPECT_DOUBLE_EQ(g_spindle.remembered_program_speed, 0.0);
    EXPECT_TRUE(g_spindle.start_request_pending);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, M4S0RetainsReverseStartRequest)
{
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CCW, 0.0));

    EXPECT_EQ(g_spindle.command_state, SPINDLE_CCW);
    EXPECT_DOUBLE_EQ(g_spindle.command_speed, 0.0);
    EXPECT_TRUE(g_spindle.start_request_pending);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, StopCancelsProgramAuthorizationAndPendingStart)
{
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW, 0.0));
    g_spindle.run_authorized = true;

    spindle_request_stop();

    EXPECT_EQ(g_spindle.command_state, SPINDLE_STOP);
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_FALSE(g_spindle.start_request_pending);
    EXPECT_EQ(g_spindle.pending_event, SPINDLE_EVENT_STOP);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, PlcDirectionDoesNotOverrideProgramAuthorization)
{
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CCW, 0.0));
    g_spindle.run_authorized = true;
    g_spindle.program_run_authorized = true;

    EXPECT_TRUE(spindle_request_manual_run(SPINDLE_CW));

    EXPECT_EQ(g_spindle.command_state, SPINDLE_CCW);
    EXPECT_EQ(g_spindle.pending_event, SPINDLE_EVENT_NONE);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, ManualDirectionRemainsAvailableWithoutProgramAuthorization)
{
    EXPECT_TRUE(spindle_request_manual_run(SPINDLE_CCW));

    EXPECT_EQ(g_spindle.command_state, SPINDLE_CCW);
    EXPECT_EQ(g_spindle.pending_event, SPINDLE_EVENT_CCW);
    EXPECT_FALSE(g_spindle.program_run_authorized);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, ProgramCommandSpeedIsIndependentOfPlcSpeedSource)
{
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW, 1000.0));
    spindle_set_plc_speed(2500.0);
    spindle_set_speed_source(SPINDLE_SPEED_SOURCE_PLC);

    EXPECT_DOUBLE_EQ(spindle_get_command_speed(), 2500.0);
    EXPECT_DOUBLE_EQ(spindle_get_program_command_speed(), 1000.0);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, ProgramCommandSpeedRemainsLatestSAfterStop)
{
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CCW, 1000.0));
    spindle_request_stop();

    EXPECT_DOUBLE_EQ(spindle_get_program_command_speed(), 1000.0);
}

TEST_F(SpindleZeroSpeedAuthorizationTest, GeneralAlarmDoesNotAbortControlledResetStop)
{
    rt_data->sys_state[SYS_ESTOP_INPUT] = false;
    rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = false;
    rt_data->sys_state[SYS_IS_ALARMING] = false;
    rt_data->axis_data[AXIS_SP].status.is_disable = false;
    rt_data->axis_data[AXIS_SP].status.is_error = false;
    ASSERT_FALSE(spindle_internal_reset_stop_abort_required());

    rt_data->sys_state[SYS_IS_ALARMING] = true;

    EXPECT_FALSE(spindle_internal_reset_stop_abort_required());
}
