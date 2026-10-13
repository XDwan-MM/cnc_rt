#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "test_env.h"
#include "global.h"
#include "auto_mst.h"
#include "driver.h"
#include "spl_control.h"
#include "task.h"
#include "plc_f_signal_handler.h"
#include <regapi.h>
#include <parapi.h>
#include <varapi.h>

// 只替代硬件控制字写入；任务、定向、主轴、自动握手、F发布运行生产代码。
extern "C" hal_retcode_t __wrap_Ethercat_Driver_Write_Controlword(hal_driver_t *drv,
                                                                uint16_t value)
{
    drv->control_word = value;
    return HAL_DRIVER_SUCCESS;
}

class TestSpindle : public ::testing::Test
{
protected:
    PLAN_BLOCK block{};

    void SetUp() override
    {
        if (!testEnvReady())
            GTEST_SKIP() << testEnvMessage();
        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        std::memset(rt_data->chan_state, 0, sizeof(rt_data->chan_state));
        rt_data->nc_event = SYS_EVENT_NONE;
        *nc_reset = {};
        spl_context = {};
        spindle_driver = {};
        spindle_driver.initialized = true;
        spindle_driver.act_mode = DRV_MODE_CSV;
        spindle_driver.req_mode = DRV_MODE_CSV;
        enabled();
        rt_data->axis_data[AXIS_SP].max_feed_acc = 100000.0; // 100 rpm/cycle
        rt_data->axis_data[AXIS_SP].spdl_max_speed = 6000.0;
        setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, 100.0, 0);
        resetRegData();
        event_queue_reset(event_queue);
        cycle_tp_mstb_cmd_data_reset();
        cycle_tp->exec_state = EXEC_RUNNING;
        cycle_tp->plan_block = &block;
        parameter(PAR_CH_TMF_DELAY, 0);
        parameter(PAR_CH_TFIN_DELEY, 2);
    }

    void TearDown() override
    {
        cycle_tp->plan_block = nullptr;
        spl_context = {};
        spindle_driver = {};
        *nc_reset = {};
        rt_data->nc_event = SYS_EVENT_NONE;
        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        cycle_tp_mstb_cmd_data_reset();
        event_queue_reset(event_queue);
    }

    void parameter(int id, int value)
    {
        ParamUnit p{};
        p.value.val_int = value;
        setParData(PARAMAN_FILE_CHAN, 0, id, p);
    }

    bool f(int byte, int bit)
    {
        bool value = false;
        getRegBit(REG_F, byte, bit, &value);
        return value;
    }

    void fin(bool value)
    {
        if (value) setRegBit(REG_G, 4, 3);
        else clearRegBit(REG_G, 4, 3);
    }

    void enabled()
    {
        spindle_driver.state = DRV_STATE_OPERATION_ENABLED;
        spindle_driver.status_word = 0x0027;
    }

    void released()
    {
        spindle_driver.orient_state = DRV_ORIENT_IDLE;
        spindle_driver.state = DRV_STATE_READY_TO_SWITCHED_ON;
        spindle_driver.status_word = 0x0021;
    }

    void rotationEvent(int direction)
    {
        PLC_EVENT event{};
        event.id = PLC_EV_MSG_SPINDLE;
        event.spindle_payload.direction = direction;
        event_queue_push(event_queue, &event);
    }

    void orientEvent(int action)
    {
        PLC_EVENT event{};
        event.id = PLC_EV_MSG_SPD_ORI;
        event.spd_ori_payload.p1 = action;
        event_queue_push(event_queue, &event);
    }

    // 与 motion thread 相同顺序：任务事件 → 主轴 → 自动指令 → F信号。
    void frame(int new_mcode = -1, int speed = 0)
    {
        task_mq_proc();
        spindle_control_update();
        if (new_mcode >= 0)
        {
            block = {};
            block.mst_data.size = 1;
            block.mst_data.m_code = new_mcode;
            block.mst_data.spindle.speed = speed;
            auto_get_mstb_cmd();
        }
        else if (block.mst_data.size)
            auto_execute_mstb_cmd();
        plc_f_signal_handler_cycle();
    }

    void oldOrientation()
    {
        spl_context.orient_phase = SPINDLE_ORIENT_DONE;
        spl_context.orient_hold_stop = true;
        spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
        spl_context.mcode_feedback = true;
        spl_context.vel_planner.current_vel = -5000.0;
        spl_context.vel_planner.target_vel = 0.0;
        spindle_driver.orient_state = DRV_ORIENT_DONE;
        spindle_driver.actual_vel = 0;
    }

    void m29Ready()
    {
        spl_context.spl_event = SPINDLE_EVENT_NONE;
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
        spl_context.m29_phase = SPINDLE_M29_SWITCHING_CSP;
        spindle_driver.orient_state = DRV_ORIENT_IDLE;
        spindle_driver.mode_switch_state = DRV_MODE_SWITCH_DONE;
        spindle_driver.act_mode = DRV_MODE_CSP;
        enabled();
    }

    void waitFinWidth()
    {
        frame(); frame(); frame();
    }
};

TEST_F(TestSpindle, OrientationReleaseStartsCwAndCcwInSameCycle)
{
    for (int m : {M03, M04})
    {
        oldOrientation();
        frame(m, 3000);
        EXPECT_FALSE(f(45, 3));
        orientEvent(-1);
        frame();
        EXPECT_FALSE(f(45, 3));
        spindle_driver.actual_vel = 2; // 驱动器释放后的实际起始速度
        released();
        frame();
        EXPECT_EQ(spl_context.spl_state, m == M03 ? SPINDLE_CW : SPINDLE_CCW);
        EXPECT_EQ(spl_context.vel_planner.current_vel, 2.0);
        EXPECT_EQ(spindle_driver.target_state, DRV_STATE_OPERATION_ENABLED);
        EXPECT_FALSE(spl_context.start_request_pending);
        EXPECT_FALSE(f(45, 3));
        enabled();
        spindle_driver.actual_vel = 0;
        for (int i = 0; i < 32; ++i)
        {
            frame();
            EXPECT_FALSE(f(45, 3));
        }
        const int target = m == M03 ? 3000 : -3000;
        spindle_driver.actual_vel = -target; // 大小相同、方向相反也不能到速
        frame();
        EXPECT_FALSE(f(45, 3));
        spindle_driver.actual_vel = target;
        frame();
        EXPECT_TRUE(f(45, 3));
        EXPECT_FALSE(f(45, 1));
    }
}

TEST_F(TestSpindle, DeferredStartBeforeAndAfterRelease)
{
    for (bool event_first : {false, true})
    {
        spl_context = {};
        oldOrientation();
        spl_context.mcp_target_speed = 3000;
        if (event_first)
        {
            rotationEvent(-1);
            frame();
            EXPECT_TRUE(spl_context.deferred_start_pending);
        }
        orientEvent(-1);
        frame();
        if (!event_first)
            rotationEvent(-1);
        released();
        frame();
        EXPECT_EQ(spl_context.spl_state, SPINDLE_CCW);
        EXPECT_EQ(spl_context.vel_planner.current_vel, 0.0);
        EXPECT_FALSE(spl_context.deferred_start_pending);
        EXPECT_FALSE(f(45, 3));
    }
}

TEST_F(TestSpindle, StartArrivingAfterReleaseCannotUseStoppedArrival)
{
    oldOrientation();
    orientEvent(-1);
    frame();
    released();
    frame();
    EXPECT_EQ(spl_context.spl_state, SPINDLE_STOP);
    frame(M04, 3000); // 自动入口在主轴周期之后，也必须立即撤销旧 ARRIVED
    EXPECT_FALSE(f(45, 3));
    frame();
    EXPECT_EQ(spl_context.spl_state, SPINDLE_CCW);
    EXPECT_FALSE(f(45, 3));
}

TEST_F(TestSpindle, RepeatedSameSpeedAndEffectiveTarget)
{
    setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, 50.0, 0);
    rt_data->axis_data[AXIS_SP].spdl_max_speed = 1000.0;
    frame(M04, 3000); // 有效目标 -1000，不能用原始 -3000 或倍率后的 -1500
    spindle_driver.actual_vel = -1000;
    for (int i = 0; i < 12; ++i) frame();
    ASSERT_TRUE(f(45, 3));
    EXPECT_EQ(spl_context.vel_planner.current_vel, -1000.0);
    frame(M04, 3000);
    EXPECT_FALSE(f(45, 3));
    EXPECT_TRUE(spl_context.start_request_pending);
    frame();
    EXPECT_TRUE(f(45, 3));
    EXPECT_FALSE(spl_context.start_request_pending);
    spindle_driver.status_word = 0x0021;
    plc_f_signal_handler_cycle();
    EXPECT_FALSE(f(45, 3));
    enabled();
    spindle_driver.act_mode = DRV_MODE_CSP;
    plc_f_signal_handler_cycle();
    EXPECT_FALSE(f(45, 3));
}

TEST_F(TestSpindle, PublisherNeverChangesArrivalAndZeroRemainsActual)
{
    spl_context.spl_vel_state = SPINDLE_VEL_RAMP;
    spl_context.mcode_feedback = false;
    spindle_driver.actual_vel = 2;
    plc_f_signal_handler_cycle();
    EXPECT_EQ(spl_context.spl_vel_state, SPINDLE_VEL_RAMP);
    EXPECT_FALSE(f(45, 3));
    EXPECT_TRUE(f(45, 1));
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spl_context.mcode_feedback = true;
    spl_context.deferred_start_pending = true;
    plc_f_signal_handler_cycle();
    EXPECT_FALSE(f(45, 3));
    EXPECT_EQ(spl_context.spl_vel_state, SPINDLE_VEL_ARRIVED);
}

TEST_F(TestSpindle, StopWaitsForActualZeroAndCancelsDeferred)
{
    frame(M03, 6000);
    spindle_driver.actual_vel = 6000;
    for (int i = 0; i < 61; ++i) frame();
    ASSERT_TRUE(f(45, 3));
    spl_context.deferred_start_pending = true;
    frame(M05);
    EXPECT_FALSE(spl_context.deferred_start_pending);
    for (int i = 0; i < 61; ++i) frame();
    EXPECT_FALSE(f(45, 3));
    spindle_driver.actual_vel = 0;
    frame();
    EXPECT_TRUE(f(45, 1));
    EXPECT_TRUE(f(45, 3));
}

TEST_F(TestSpindle, SafetyAndNewOrientationCancelDeferredStarts)
{
    for (int reason = 0; reason < 5; ++reason)
    {
        spl_context = {};
        oldOrientation();
        spl_context.mcp_target_speed = 3000;
        rotationEvent(-1);
        frame();
        ASSERT_TRUE(spl_context.deferred_start_pending);
        if (reason == 0) rt_data->nc_event = SYS_EVENT_HMI_RESET;
        if (reason == 1) rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = true;
        if (reason == 2) spindle_driver.status_word = 0x0008;
        if (reason == 3) orientEvent(1);
        if (reason == 4) frame(M05);
        frame();
        EXPECT_FALSE(spl_context.deferred_start_pending);
        EXPECT_FALSE(spl_context.start_request_pending);
        rt_data->nc_event = SYS_EVENT_NONE;
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = false;
        released();
        frame();
        EXPECT_EQ(spl_context.spl_state, SPINDLE_STOP);
        EXPECT_NE(spindle_driver.target_vel, -3000);
    }
}

TEST_F(TestSpindle, CsvRequestDoesNotForgeActualMode)
{
    spindle_driver.act_mode = DRV_MODE_NONE;
    frame(M03, 3000);
    frame();
    EXPECT_EQ(spindle_driver.act_mode, DRV_MODE_NONE);
    EXPECT_EQ(spindle_driver.req_mode, DRV_MODE_CSV);
    EXPECT_FALSE(f(45, 3));
}

TEST_F(TestSpindle, M29LatchesEarlyFinAndKeepsFixedCspTarget)
{
    frame(M29);
    frame(); // FIN低并选通
    fin(true);
    waitFinWidth();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.m29_fin_latched);
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_TRUE(f(7, 0));
    fin(false); // 锁存不因PLC脉冲结束而丢失
    frame();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.m29_fin_latched);
    m29Ready();
    spindle_driver.target_pos = 10000;
    spindle_driver.actual_pos = 10052;
    frame();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_FALSE(f(7, 0));
    EXPECT_FALSE(spl_context.m29_active);
    EXPECT_FALSE(f(76, 3));
    spindle_driver.actual_pos = 10051;
    frame();
    EXPECT_EQ(spindle_driver.target_pos, 10000);
    EXPECT_EQ(spindle_driver.target_state, DRV_STATE_OPERATION_ENABLED);
}

TEST_F(TestSpindle, M29NcFirstStillWaitsForQualifiedPlcFin)
{
    frame(M29);
    frame();
    m29Ready();
    frame();
    EXPECT_TRUE(spindle_m29_is_ready());
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    fin(true);
    frame(); frame();
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    frame();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
}

TEST_F(TestSpindle, M29RejectsPreviousFinAndShortPulse)
{
    fin(true);
    frame(M29);
    m29Ready();
    waitFinWidth();
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.m29_fin_latched);
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    fin(false); frame();
    fin(true); frame(); frame();
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    fin(false); frame();
    EXPECT_EQ(cycle_tp->mstb_cmd_data.tfin_count, 0);
    fin(true); waitFinWidth();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
}

TEST_F(TestSpindle, M29RejectsFinBeforeStrobeEvenAfterLow)
{
    parameter(PAR_CH_TMF_DELAY, 5);
    frame(M29);
    frame(); // 低电平，尚未选通
    fin(true);
    for (int i = 0; i < 12; ++i) frame();
    EXPECT_TRUE(f(7, 0));
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.m29_fin_latched);
    fin(false); frame();
    fin(true); waitFinWidth();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.m29_fin_latched);
}

TEST_F(TestSpindle, M29FaultAndResetCannotAdvanceWithLatchedFin)
{
    for (int reason = 0; reason < 3; ++reason)
    {
        frame(M29); frame();
        fin(true); waitFinWidth();
        ASSERT_TRUE(cycle_tp->mstb_cmd_data.m29_fin_latched);
        m29Ready();
        if (reason == 0) spindle_driver.status_word = 0x0008;
        if (reason == 1) rt_data->nc_event = SYS_EVENT_HMI_RESET;
        if (reason == 2) spl_context.m29_phase = SPINDLE_M29_ERROR;
        frame();
        EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
        EXPECT_FALSE(cycle_tp->mstb_cmd_data.m29_fin_latched);
        cycle_tp->exec_state = EXEC_RESET;
        auto_update_mstb_cmd();
        EXPECT_FALSE(cycle_tp->mstb_cmd_data.m29_fin_low_seen);
        EXPECT_FALSE(f(7, 0));
        rt_data->nc_event = SYS_EVENT_NONE;
        cycle_tp->exec_state = EXEC_RUNNING;
        enabled();
        fin(false);
    }
}

TEST_F(TestSpindle, M29DoesNotWriteDedicatedF763)
{
    setRegBit(REG_F, 76, 3);
    spindle_request_m29();
    EXPECT_TRUE(f(76, 3));
    m29Ready();
    spindle_control_update();
    EXPECT_TRUE(f(76, 3));
    spindle_complete_m29();
    EXPECT_TRUE(f(76, 3));
    clearRegBit(REG_F, 76, 3);
    spindle_control_update();
    EXPECT_FALSE(f(76, 3));
}

TEST_F(TestSpindle, M29FullOrientationReleaseAndModeConfirmation)
{
    frame(M29);
    frame();
    EXPECT_EQ(spl_context.m29_phase, SPINDLE_M29_ORIENTING);
    EXPECT_EQ(spindle_driver.orient_state, DRV_ORIENT_REQUESTED);
    spindle_driver.orient_state = DRV_ORIENT_DONE;
    frame();
    EXPECT_EQ(spl_context.m29_phase, SPINDLE_M29_RELEASING);
    spindle_driver.actual_pos = 12345;
    spindle_driver.actual_vel = 0;
    released();
    frame();
    EXPECT_EQ(spl_context.m29_phase, SPINDLE_M29_SWITCHING_CSP);
    EXPECT_EQ(spindle_driver.target_pos, 12345);
    EXPECT_EQ(spindle_driver.mode_switch_target, DRV_MODE_CSP);
    EXPECT_FALSE(spindle_m29_is_ready());
    spindle_driver.mode_switch_state = DRV_MODE_SWITCH_DONE;
    spindle_driver.act_mode = DRV_MODE_CSP;
    frame();
    EXPECT_FALSE(spindle_m29_is_ready());
    enabled();
    spindle_driver.actual_pos = 12397;
    frame();
    EXPECT_TRUE(spindle_m29_is_ready());
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_EQ(spindle_driver.target_pos, 12345);
    fin(true); waitFinWidth();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_EQ(spindle_driver.target_pos, 12345);
}

TEST_F(TestSpindle, RotationAfterM29WaitsForActualCsv)
{
    frame(M29); frame();
    m29Ready();
    frame();
    fin(true); waitFinWidth();
    ASSERT_TRUE(cycle_tp->mstb_cmd_data.finished);
    fin(false);
    frame(M04, 3000);
    frame();
    EXPECT_TRUE(spl_context.csv_switch_pending);
    EXPECT_EQ(spindle_driver.mode_switch_target, DRV_MODE_CSV);
    EXPECT_FALSE(f(45, 3));
    spindle_driver.mode_switch_state = DRV_MODE_SWITCH_DONE;
    spindle_driver.act_mode = DRV_MODE_CSV;
    enabled();
    frame(); frame();
    EXPECT_FALSE(spl_context.csv_switch_pending);
    EXPECT_EQ(spl_context.spl_state, SPINDLE_CCW);
    EXPECT_FALSE(f(45, 3));
    spindle_driver.actual_vel = -3000;
    for (int i = 0; i < 31; ++i) frame();
    EXPECT_TRUE(f(45, 3));
}

TEST_F(TestSpindle, OriginalSixCommandsWithEarlyM29Fin)
{
    const int codes[] = {M03, M05, M04};
    const int speeds[] = {6000, 0, 5000};
    const int targets[] = {6000, 0, -5000};
    for (int n = 0; n < 3; ++n)
    {
        fin(false);
        frame(codes[n], speeds[n]);
        enabled();
        spindle_driver.actual_vel = targets[n];
        for (int i = 0; i < 62; ++i) frame();
        ASSERT_TRUE(f(45, 3));
        fin(true); waitFinWidth();
        ASSERT_TRUE(cycle_tp->mstb_cmd_data.finished);
    }
    fin(false);
    frame(M19);
    orientEvent(1); frame();
    spindle_driver.actual_vel = 0;
    spindle_driver.orient_state = DRV_ORIENT_DONE;
    frame();
    ASSERT_TRUE(f(45, 7));
    EXPECT_FALSE(f(45, 3));
    fin(true); waitFinWidth();
    ASSERT_TRUE(cycle_tp->mstb_cmd_data.finished);
    fin(false);
    frame(M04, 3000);
    orientEvent(-1); frame();
    released(); frame();
    EXPECT_EQ(spl_context.spl_state, SPINDLE_CCW);
    enabled();
    for (int i = 0; i < 31; ++i)
    {
        frame();
        EXPECT_FALSE(f(45, 3));
        EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    }
    spindle_driver.actual_vel = -3000;
    frame();
    ASSERT_TRUE(f(45, 3));
    fin(true); waitFinWidth();
    ASSERT_TRUE(cycle_tp->mstb_cmd_data.finished);
    fin(false);
    frame(M29); frame();
    fin(true); waitFinWidth();
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.m29_fin_latched);
    spindle_driver.actual_vel = 0;
    spindle_driver.orient_state = DRV_ORIENT_DONE;
    frame();
    spindle_driver.actual_pos = 32100;
    released(); frame();
    EXPECT_EQ(spl_context.m29_phase, SPINDLE_M29_SWITCHING_CSP);
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    m29Ready();
    spindle_driver.actual_pos += 52;
    frame();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_EQ(spindle_driver.target_pos, 32100);
}
