#include <gtest/gtest.h>

#include <cstring>

#include "global.h"
#include "plc_g_signal_scanner.h"
#include "plc_x_signal_scanner.h"
#include "regapi.h"
#include "task.h"
#include "test_env.h"

namespace {

class TestPlcXSignalScanner : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady())
        {
            GTEST_SKIP() << testEnvMessage();
        }

        ASSERT_NE(event_queue, nullptr);
        RegData *registers = getRegDataPtr();
        ASSERT_NE(registers, nullptr);
        event_queue_reset(event_queue);
        std::memset(registers->regX, static_cast<char>(0xFF), REG_X_NUM);
        std::memset(registers->regG, static_cast<char>(0xFF), REG_G_NUM);
        rt_set_sys_state(SYS_ESTOP_INPUT, false);
    }
};

TEST_F(TestPlcXSignalScanner, XEstopDoesNotDispatchResetOnRelease)
{
    /* 首次扫描：X8.4 释放。 */
    plc_x_signal_scan_cycle();
    EXPECT_FALSE(rt_get_sys_state(SYS_ESTOP_INPUT));
    EXPECT_TRUE(event_queue_is_empty(event_queue));

    /* X8.4 低电平有效，按下后进入急停。 */
    setRegValue8(REG_X, PLC_X_ESTOP_REG, 0);
    plc_x_signal_scan_cycle();
    EXPECT_TRUE(rt_get_sys_state(SYS_ESTOP_INPUT));
    EXPECT_TRUE(event_queue_is_empty(event_queue));

    /* X8.4 释放不修改急停输入，也不触发复位事件；G8.4 保持现有复位逻辑。 */
    setRegValue8(REG_X, PLC_X_ESTOP_REG, static_cast<char>(1 << PLC_X_ESTOP_BIT));
    plc_x_signal_scan_cycle();
    EXPECT_TRUE(rt_get_sys_state(SYS_ESTOP_INPUT));
    EXPECT_TRUE(event_queue_is_empty(event_queue));

    /* 保持释放状态不应重复处理。 */
    plc_x_signal_scan_cycle();
    EXPECT_TRUE(rt_get_sys_state(SYS_ESTOP_INPUT));
    EXPECT_TRUE(event_queue_is_empty(event_queue));
}

TEST_F(TestPlcXSignalScanner, GEstopKeepsResetEventBehavior)
{
    /* 先建立 G8.4 的释放快照；首次扫描可能产生其他初始化事件。 */
    plc_g_signal_scan_cycle();
    event_queue_reset(event_queue);

    /* G8.4 按下时置起系统急停输入，不发送复位事件。 */
    setRegValue8(REG_G, PLC_X_ESTOP_REG, static_cast<char>(0xEF));
    plc_g_signal_scan_cycle();
    EXPECT_TRUE(rt_get_sys_state(SYS_ESTOP_INPUT));
    EXPECT_TRUE(event_queue_is_empty(event_queue));

    /* G8.4 由 0 变 1 时清除急停并发送一次 reset_state=1、reset_type=0。 */
    setRegValue8(REG_G, PLC_X_ESTOP_REG, static_cast<char>(0xFF));
    plc_g_signal_scan_cycle();
    EXPECT_FALSE(rt_get_sys_state(SYS_ESTOP_INPUT));
    ASSERT_EQ(event_queue_get_size(event_queue), 1);

    PLC_EVENT event;
    ASSERT_EQ(event_queue_pop(event_queue, &event), GNC_OPERATOR_SUCCESS);
    EXPECT_EQ(event.id, PLC_EV_MSG_RESET);
    EXPECT_EQ(event.reset_payload.reset_state, 1);
    EXPECT_EQ(event.reset_payload.reset_type, 0);

    /* 急停保持释放时不重复发送复位事件。 */
    plc_g_signal_scan_cycle();
    EXPECT_TRUE(event_queue_is_empty(event_queue));
}

TEST_F(TestPlcXSignalScanner, G23Dot3ControlsMpgExternalDecelerationPermission)
{
    /* 建立 G23.3 的初始快照，随后验证低电平和高电平事件。 */
    plc_g_signal_scan_cycle();
    event_queue_reset(event_queue);
    rt_data->sys_state[SYS_IS_MPG_EX_DEC_ENABLE] = false;

    setRegValue8(REG_G, 23, static_cast<char>(0xF7));
    plc_g_signal_scan_cycle();
    ASSERT_EQ(event_queue_get_size(event_queue), 1);
    ASSERT_EQ(task_mq_proc(), GNC_OPERATOR_SUCCESS);
    EXPECT_FALSE(rt_data->sys_state[SYS_IS_MPG_EX_DEC_ENABLE]);

    setRegValue8(REG_G, 23, static_cast<char>(0xFF));
    plc_g_signal_scan_cycle();
    ASSERT_EQ(event_queue_get_size(event_queue), 1);
    ASSERT_EQ(task_mq_proc(), GNC_OPERATOR_SUCCESS);
    EXPECT_TRUE(rt_data->sys_state[SYS_IS_MPG_EX_DEC_ENABLE]);

    plc_g_signal_scan_cycle();
    EXPECT_TRUE(event_queue_is_empty(event_queue));
}

} // namespace
