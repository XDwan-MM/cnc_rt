#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "global.h"
#include "jog.h"
#include "plc_event_types.h"
#include "rt_data.h"
#include "task.h"
#include "test_env.h"
#include <varapi.h>

namespace
{

class TestJog : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady())
        {
            GTEST_SKIP() << testEnvMessage();
        }

        event_queue_reset(event_queue);
        jog_handle_estop();
        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        std::memset(rt_data->chan_data, 0, sizeof(rt_data->chan_data));
        rt_data->nc_event = SYS_EVENT_NONE;
        rt_data->current_mode = MODE_JOG;
        rt_data->target_mode = MODE_JOG;

        for (int axis = 0; axis < JOG_AXIS_COUNT; axis++)
        {
            rt_data->axis_data[axis].plmt = 1000.0;
            rt_data->axis_data[axis].nlmt = -1000.0;
            rt_data->axis_data[axis].jog_lowspd = 10.0;
            rt_data->axis_data[axis].jog_fastspd = 20.0;
            rt_data->axis_data[axis].max_feed_acc = 100.0;
            rt_data->axis_data[axis].max_rapid_acc = 100.0;
            rt_data->axis_data[axis].rpd_ex_dec1_vel = 3.0;
            rt_data->axis_data[axis].rpd_ex_dec2_vel = 2.0;
            rt_data->axis_data[axis].rpd_ex_dec3_vel = 1.0;
            rt_data->axis_data[axis].rpd_ex_dec4_vel = 4.0;
            rt_data->axis_data[axis].rpd_ex_dec5_vel = 5.0;
        }

        rt_data->chan_data[0].ex_dec_enable = true;
        rt_data->chan_data[0].ex_dec2_enable = true;
        rt_data->chan_data[0].ex_dec3_enable = true;
        rt_data->chan_data[0].ex_dec4_enable = true;
        rt_data->chan_data[0].ex_dec5_enable = true;
        setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, 100.0, 0);
        setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, 100.0, 0);

        /* 首周期只完成 JOG 进入同步。 */
        handle_jog_cmd();
    }

    void TearDown() override
    {
        if (rt_data != nullptr)
        {
            jog_handle_estop();
            rt_data->nc_event = SYS_EVENT_NONE;
            rt_data->sys_state[SYS_IS_RAPID] = false;
        }
    }

    void cycle(SYS_EVENT event = SYS_EVENT_NONE)
    {
        rt_data->nc_event = event;
        handle_jog_cmd();
    }

    void run_cycles(int count)
    {
        for (int i = 0; i < count; i++)
            cycle();
    }

    void send_external_decel_event(int function, int axis, int direction, bool active)
    {
        PLC_EVENT event = {};

        event.id = PLC_EV_MSG_EXTERNAL_DECEL;
        event.external_decel_payload.function = function;
        event.external_decel_payload.axis_num = axis;
        event.external_decel_payload.direction = direction;
        event.external_decel_payload.active = active ? 1 : 0;
        ASSERT_EQ(event_queue_push(event_queue, &event), GNC_OPERATOR_SUCCESS);
        ASSERT_EQ(task_mq_proc(), GNC_OPERATOR_SUCCESS);
    }
};

} // namespace

TEST_F(TestJog, PressAndReleasePreserveVelocityRamp)
{
    cycle(SYS_EVENT_X_POS_PRESS);
    EXPECT_GT(jog_get_axis_velocity(AXIS_X), 0.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, jog_get_axis_velocity(AXIS_X));

    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 10.0, 1.0e-6);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, jog_get_axis_velocity(AXIS_X));

    cycle(SYS_EVENT_X_POS_RELEASE);
    EXPECT_LT(jog_get_axis_velocity(AXIS_X), 10.0);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, jog_get_axis_velocity(AXIS_X));
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_X].cmd_vel, 0.0);
}

TEST_F(TestJog, ExternalDecelerationUsesRpdVelocityForNormalAndRapidJog)
{
    rt_data->axis_data[AXIS_X].rpd_ex_dec1_vel = 3.0;
    send_external_decel_event(PLC_EXTERNAL_DECEL_FUNCTION_1, AXIS_X, JOG_DIR_POSITIVE, true);

    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 3.0, 1.0e-6);

    cycle(SYS_EVENT_X_POS_RELEASE);
    run_cycles(120);
    rt_data->sys_state[SYS_IS_RAPID] = true;

    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 3.0, 1.0e-6);
}

TEST_F(TestJog, MultipleExternalDecelerationSignalsUseMinimumAndMatchDirection)
{
    send_external_decel_event(PLC_EXTERNAL_DECEL_FUNCTION_1, AXIS_X, JOG_DIR_POSITIVE, true);
    send_external_decel_event(PLC_EXTERNAL_DECEL_FUNCTION_2, AXIS_X, JOG_DIR_POSITIVE, true);
    send_external_decel_event(PLC_EXTERNAL_DECEL_FUNCTION_3, AXIS_X, JOG_DIR_NEGATIVE, true);

    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 2.0, 1.0e-6);

    send_external_decel_event(PLC_EXTERNAL_DECEL_FUNCTION_2, AXIS_X, JOG_DIR_POSITIVE, false);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 3.0, 1.0e-6);
}

TEST_F(TestJog, InvalidExternalDecelerationEventsAreIgnored)
{
    send_external_decel_event(0, AXIS_X, JOG_DIR_POSITIVE, true);
    send_external_decel_event(6, AXIS_X, JOG_DIR_POSITIVE, true);
    send_external_decel_event(1, -1, JOG_DIR_POSITIVE, true);
    send_external_decel_event(1, AXIS_X, JOG_DIR_NONE, true);

    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 10.0, 1.0e-6);
}

TEST_F(TestJog, HardwareLimitBlocksOnlyMatchingDirection)
{
    rt_data->axis_data[AXIS_X].status.is_fwd_limit = true;

    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(30);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);

    cycle(SYS_EVENT_X_POS_RELEASE);
    run_cycles(30);
    cycle(SYS_EVENT_X_NEG_PRESS);
    run_cycles(30);
    EXPECT_LT(jog_get_axis_velocity(AXIS_X), 0.0);
}

TEST_F(TestJog, InvalidAccelerationDoesNotLeaveAxisRunning)
{
    rt_data->axis_data[AXIS_X].max_feed_acc = 0.0;

    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(10);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);

    /* 参数恢复后，仍保持按键请求，下一周期即可正常启动。 */
    rt_data->axis_data[AXIS_X].max_feed_acc = 100.0;
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 10.0, 1.0e-6);
}

TEST_F(TestJog, FatalAlarmRequiresResetAndModeSwitchRequestsDeceleration)
{
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(60);
    double velocity_before_stop = jog_get_axis_velocity(AXIS_X);
    ASSERT_GT(velocity_before_stop, 0.0);

    rt_data->sys_state[SYS_IS_ALARMING] = true;
    rt_data->axis_data[AXIS_X].status.is_error = true;
    cycle();
    EXPECT_LT(jog_get_axis_velocity(AXIS_X), velocity_before_stop);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);

    rt_data->sys_state[SYS_IS_ALARMING] = false;
    rt_data->axis_data[AXIS_X].status.is_error = false;
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(60);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);

    /* 报警解除后，必须先 RESET 才能再次接受 JOG 请求。 */
    cycle(SYS_EVENT_HMI_RESET);
    run_cycles(2);
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(60);
    velocity_before_stop = jog_get_axis_velocity(AXIS_X);
    ASSERT_GT(velocity_before_stop, 0.0);

    rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
    cycle();
    EXPECT_LT(jog_get_axis_velocity(AXIS_X), velocity_before_stop);
}

TEST_F(TestJog, ModeEntryCannotBypassFatalAlarmReset)
{
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    ASSERT_GT(jog_get_axis_velocity(AXIS_X), 0.0);

    rt_data->sys_state[SYS_IS_ALARMING] = true;
    rt_data->axis_data[AXIS_X].status.is_error = true;
    cycle();
    run_cycles(120);

    rt_data->sys_state[SYS_IS_ALARMING] = false;
    rt_data->axis_data[AXIS_X].status.is_error = false;
    rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
    cycle(SYS_EVENT_X_POS_PRESS);
    rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
    cycle();
    run_cycles(10);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);

    cycle(SYS_EVENT_HMI_RESET);
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    EXPECT_GT(jog_get_axis_velocity(AXIS_X), 0.0);
}

TEST_F(TestJog, EstopAndResetClearMotionRequest)
{
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(60);
    ASSERT_GT(jog_get_axis_velocity(AXIS_X), 0.0);

    jog_handle_estop();
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);

    rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = false;
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(60);
    ASSERT_GT(jog_get_axis_velocity(AXIS_X), 0.0);
    cycle(SYS_EVENT_HMI_RESET);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 0.0, 1.0e-9);
}

TEST_F(TestJog, PlcExternalDecelerationEventReachesJog)
{
    cycle(SYS_EVENT_X_POS_PRESS);
    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 10.0, 1.0e-6);

    send_external_decel_event(PLC_EXTERNAL_DECEL_FUNCTION_1, AXIS_X, JOG_DIR_POSITIVE, true);
    EXPECT_EQ(rt_data->nc_event, SYS_EVENT_EXTERNAL_DECEL_POS_ON);

    run_cycles(120);
    EXPECT_NEAR(jog_get_axis_velocity(AXIS_X), 3.0, 1.0e-6);
}
