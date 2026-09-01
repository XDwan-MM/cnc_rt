#include <gtest/gtest.h>
#include <cstring>
#include "test_env.h"
#include "auto.h"
#include "datatype.h"
#include "global.h"
#include "pathplan.h"
#include "auto_mst.h"
#include <parapi.h>
#include <regapi.h>
#include <varapi.h>

/********************************************************************
 * @file        tst_auto.cpp
 * @brief       src/motion/auto 自动模式单元测试
 ********************************************************************/

class TestAuto : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady()) {
            GTEST_SKIP() << testEnvMessage();
        }
        seed_motion_params();
        reset_auto_fixture();
    }

    void set_ncu_param_uint(int index, unsigned int value)
    {
        ParamUnit p = {};
        p.value.val_uint = value;
        setParData(PARAMAN_FILE_NCU, 0, index, p);
    }

    void set_chan_param_int(int index, int value)
    {
        ParamUnit p = {};
        p.value.val_int = value;
        setParData(PARAMAN_FILE_CHAN, 0, index, p);
    }

    void set_chan_param_real(int index, double value)
    {
        ParamUnit p = {};
        p.value.val_real = value;
        setParData(PARAMAN_FILE_CHAN, 0, index, p);
    }

    void set_axis_param_real(int axis, int index, double value)
    {
        ParamUnit p = {};
        p.value.val_real = value;
        setParData(PARAMAN_FILE_AXIS, axis, index, p);
    }

    void seed_motion_params()
    {
        set_ncu_param_uint(PAR_NCU_CYCLE, 1000);
        set_chan_param_int(PAR_CH_VPLAN_MODE, T_SHAPE);
        set_chan_param_real(PAR_CH_DRYRUN_SPD, 20.0);
        set_chan_param_real(PAR_CH_PATH_TOLERANCE, 0.01);

        for (int i = 0; i < SYS_AXES_NUM; i++) {
            set_axis_param_real(i, PAR_AX_JOG_LOWSPD, 100.0);
            set_axis_param_real(i, PAR_AX_RAPID_SPD, 100.0);
            set_axis_param_real(i, PAR_AX_RAPID_ACC, 1000.0);
            set_axis_param_real(i, PAR_AX_RAPID_JK, 10000.0);
            set_axis_param_real(i, PAR_AX_FEED_SPD, 80.0);
            set_axis_param_real(i, PAR_AX_FEED_ACC, 800.0);
            set_axis_param_real(i, PAR_AX_FEED_JK, 8000.0);
            set_axis_param_real(i, PAR_AX_CORNER_VEL_DIFF, 10.0);
            set_axis_param_real(i, PAR_AX_POS_TOL, 0.01);
            set_axis_param_real(i, PAR_AX_PLMT, 1000.0);
            set_axis_param_real(i, PAR_AX_NLMT, -1000.0);
        }
    }

    static PLAN_BLOCK make_motion_block(INTERP_TYPE type, pmc_mode mode, int line_number, double end_x)
    {
        PLAN_BLOCK block = {};
        block.type = type;
        block.mode = mode;
        block.line_number = line_number;
        block.feed_vel = (type == RAPID) ? 100.0 : 60.0;
        block.max_vel = block.feed_vel;
        block.max_acc = 800.0;
        block.max_jerk = 8000.0;
        block.start_pos = (POS){.x = 0.0, .y = 0.0, .z = 0.0};
        block.end_pos = (POS){.x = end_x, .y = 0.0, .z = 0.0};
        block_cal_vector(&block);
        block_cal_length(&block);
        return block;
    }

    static PLAN_BLOCK make_dwell_block(int line_number, int pause_cycles)
    {
        PLAN_BLOCK block = {};
        block.mode = PMC_MODE_PAUSE;
        block.line_number = line_number;
        block.pause_time = pause_cycles;
        return block;
    }

    static PLAN_BLOCK make_mst_block(int line_number, int m_code, int size = 1,
                                     int spindle_speed = 0, int tool_number = 0)
    {
        PLAN_BLOCK block = {};
        block.mode = PMC_MODE_M_CMD;
        block.line_number = line_number;
        block.mst_data.size = size;
        block.mst_data.m_code = m_code;
        block.mst_data.spindle.speed = spindle_speed;
        block.mst_data.tool_number = tool_number;
        return block;
    }

    void reset_auto_fixture()
    {
        cycle_tp_reset();
        queue_reset(parser_queue);
        queue_reset(plan_queue);
        queue_reset(pretreat_queue);

        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        std::memset(rt_data->chan_state, 0, sizeof(rt_data->chan_state));
        std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        std::memset(rt_data->chan_data, 0, sizeof(rt_data->chan_data));
        rt_data->nc_event = SYS_EVENT_NONE;
        rt_data->target_mode = MODE_MEM;
        rt_data->current_mode = MODE_MEM;
        rt_data->path_data.parser_error_no = -1;
        rt_data->path_data.parser_error_line = 0;

        resetRegData();
        setVarChanInt(0, VAR_CHAN_PART_CNTR, 0, 0);
        setVarChanInt(0, VAR_CHAN_FIN_SIGNAL, 0, 0);
        set_chan_param_int(PAR_CH_TMF_DELAY, 0);
        set_chan_param_int(PAR_CH_TFIN_DELEY, 0);

        setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 0.0, 0);
        setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, 100.0, 0);
        setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, 100.0, 0);
        setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
        setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
    }

    void start_with_block(const PLAN_BLOCK& block)
    {
        ASSERT_EQ(queue_push(pretreat_queue, &block), GNC_OPERATOR_SUCCESS);
        cycle_tp->exec_state = EXEC_RUNNING;
        cycle_tp->running_stage = PROGRAM_START;
        rt_data->chan_state[0][CHAN_IS_PARSERING] = true;
        handle_auto_cmd();
    }

    void send_event(SYS_EVENT event)
    {
        rt_data->nc_event = event;
        handle_auto_cmd();
        rt_data->nc_event = SYS_EVENT_NONE;
    }

    void finish_current_mst()
    {
        ASSERT_EQ(cycle_tp->exec_cmd, DO_MSTB_CMD);
        setRegBit(REG_G, 4, 3);
        handle_auto_cmd();
    }

    static char read_reg8(RegType type, int index)
    {
        char value = 0;
        EXPECT_EQ(getRegValue8(type, index, &value), 0);
        return value;
    }

    static bool read_reg_bit(RegType type, int index, int bit)
    {
        bool value = false;
        EXPECT_EQ(getRegBit(type, index, bit, &value), 0);
        return value;
    }

    void run_until_state(EXEC_STATE state, int max_cycles = 2000)
    {
        for (int i = 0; i < max_cycles && cycle_tp->exec_state != state; ++i) {
            rt_data->nc_event = SYS_EVENT_NONE;
            handle_auto_cmd();
        }
        ASSERT_EQ(cycle_tp->exec_state, state);
    }
};

TEST_F(TestAuto, CycleTpInitAllocatesDefaults)
{
    ASSERT_NE(cycle_tp, nullptr);
    ASSERT_NE(cycle_tp->plan_block, nullptr);
    ASSERT_NE(cycle_tp->pos_cmd_data.vel_profile, nullptr);
    ASSERT_NE(cycle_tp->pos_cmd_data.traj_data, nullptr);
    EXPECT_EQ(cycle_tp->exec_state, EXEC_IDLE);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_NULL_CMD);
    EXPECT_EQ(cycle_tp->plan_block->type, UNKNOWN);
}

TEST_F(TestAuto, CycleTpResetClearsMotionStateButKeepsBuffers)
{
    cycle_tp->exec_state = EXEC_RUNNING;
    cycle_tp->exec_cmd = DO_CHOP_CMD;
    cycle_tp->plan_block->type = LINE;
    cycle_tp->pos_cmd_data.traj_data->s = 12.0;
    cycle_tp->pos_cmd_data.traj_data->v = 5.0;
    cycle_tp->pos_cmd_data.interp_cycle = 99;
    cycle_tp->pos_cmd_data.vel_profile->t_cycle = 20.0;
    cycle_tp->pos_cmd_data.accum_distance = 3.0;

    VEL_PROFILE* vel_profile = cycle_tp->pos_cmd_data.vel_profile;
    TRAJ_PARAMS* traj_data = cycle_tp->pos_cmd_data.traj_data;

    cycle_tp_reset();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_IDLE);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_NULL_CMD);
    EXPECT_EQ(cycle_tp->plan_block->type, UNKNOWN);
    EXPECT_EQ(cycle_tp->pos_cmd_data.vel_profile, vel_profile);
    EXPECT_EQ(cycle_tp->pos_cmd_data.traj_data, traj_data);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.traj_data->s, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.vel_profile->t_cycle, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.accum_distance, 0.0));
}

TEST_F(TestAuto, RerunStartsParserFromBeginning)
{
    rt_data->nc_event = SYS_EVENT_RERUN;

    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_EQ(cycle_tp->running_stage, PROGRAM_WAIT);
    EXPECT_TRUE(rt_get_chan_state(0, CHAN_IS_PARSER_RERUN));
    EXPECT_TRUE(rt_get_chan_state(0, CHAN_IS_PARSERING));

    int is_running = 0;
    int is_moving = 0;
    int run_row = -1;
    getVarChanInt(0, VAR_CHAN_IS_RUNNING, &is_running, 0);
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    getVarChanInt(0, VAR_CHAN_RUN_ROW, &run_row, 0);
    EXPECT_EQ(is_running, 1);
    EXPECT_EQ(is_moving, 1);
    EXPECT_EQ(run_row, 0);
}

TEST_F(TestAuto, PauseStopsCurrentMotionWithoutPoppingBlock)
{
    PLAN_BLOCK block = make_motion_block(LINE, PMC_MODE_CHOP, 10, 100.0);
    start_with_block(block);
    ASSERT_FALSE(queue_is_empty(plan_queue));

    rt_data->nc_event = SYS_EVENT_PAUSE;
    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_PAUSING);
    EXPECT_EQ(cycle_tp->running_stage, BLOCK_PAUSED);
    EXPECT_FALSE(queue_is_empty(plan_queue));

    int is_moving = 1;
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    EXPECT_EQ(is_moving, 0);
}

TEST_F(TestAuto, RerunAfterPauseContinuesCurrentBlock)
{
    PLAN_BLOCK block = make_motion_block(LINE, PMC_MODE_CHOP, 11, 100.0);
    start_with_block(block);

    rt_data->nc_event = SYS_EVENT_PAUSE;
    handle_auto_cmd();
    ASSERT_EQ(cycle_tp->exec_state, EXEC_PAUSING);
    ASSERT_FALSE(queue_is_empty(plan_queue));

    rt_data->nc_event = SYS_EVENT_RERUN;
    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_FALSE(queue_is_empty(plan_queue));
    EXPECT_EQ(queue_peak(plan_queue)->line_number, 11);
}

TEST_F(TestAuto, PauseInterruptsDwellWithoutPoppingBlock)
{
    start_with_block(make_dwell_block(11, 20));
    ASSERT_EQ(cycle_tp->exec_cmd, DO_DWELL_CMD);
    ASSERT_FALSE(queue_is_empty(plan_queue));

    send_event(SYS_EVENT_PAUSE);

    EXPECT_EQ(cycle_tp->exec_state, EXEC_PAUSING);
    EXPECT_EQ(cycle_tp->running_stage, BLOCK_PAUSED);
    EXPECT_FALSE(queue_is_empty(plan_queue));
    int is_moving = 1;
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    EXPECT_EQ(is_moving, 0);
}

TEST_F(TestAuto, RerunAfterDwellPauseKeepsRemainingDwell)
{
    start_with_block(make_dwell_block(12, 20));
    send_event(SYS_EVENT_PAUSE);
    ASSERT_EQ(cycle_tp->exec_state, EXEC_PAUSING);
    ASSERT_EQ(cycle_tp->running_stage, BLOCK_PAUSED);
    ASSERT_EQ(queue_peak(plan_queue)->pause_time, 19);

    send_event(SYS_EVENT_RERUN);

    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_DWELL_CMD);
    EXPECT_EQ(cycle_tp->dwell_cmd_data.dwell_cycle, 19);
    EXPECT_EQ(cycle_tp->plan_block->pause_time, 19);
    EXPECT_FALSE(queue_is_empty(plan_queue));
}

TEST_F(TestAuto, ResetClearsQueuesAndNextStartRerunsParser)
{
    PLAN_BLOCK block = make_motion_block(LINE, PMC_MODE_CHOP, 12, 100.0);
    start_with_block(block);

    rt_data->nc_event = SYS_EVENT_HMI_RESET;
    handle_auto_cmd();
    for (int i = 0; i < 10000 && cycle_tp->exec_state != EXEC_IDLE; i++) {
        rt_data->nc_event = SYS_EVENT_NONE;
        handle_auto_cmd();
    }

    EXPECT_EQ(cycle_tp->exec_state, EXEC_IDLE);
    EXPECT_TRUE(queue_is_empty(parser_queue));
    EXPECT_TRUE(queue_is_empty(pretreat_queue));
    EXPECT_FALSE(queue_is_empty(plan_queue));
    EXPECT_TRUE(rt_get_chan_state(0, CHAN_IS_PARSERING));

    rt_data->nc_event = SYS_EVENT_RERUN;
    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_TRUE(rt_get_chan_state(0, CHAN_IS_PARSER_RERUN));
    EXPECT_TRUE(rt_get_chan_state(0, CHAN_IS_PARSERING));
}

TEST_F(TestAuto, DryRunReplansCurrentFeedBlock)
{
    PLAN_BLOCK block = make_motion_block(LINE, PMC_MODE_CHOP, 13, 100.0);
    start_with_block(block);
    ASSERT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 60.0));

    send_event(SYS_EVENT_DRY_RUN_ON);

    EXPECT_TRUE(rt_get_sys_state(SYS_IS_DRY_RUN));
    EXPECT_EQ(cycle_tp->trigger_event, DRY_RUN_ENABLE);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 20.0));

    send_event(SYS_EVENT_DRY_RUN_OFF);

    EXPECT_FALSE(rt_get_sys_state(SYS_IS_DRY_RUN));
    EXPECT_EQ(cycle_tp->trigger_event, NONE_EVENT);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 60.0));
}

TEST_F(TestAuto, SingleBlockPausesAfterOneCompletedBlock)
{
    send_event(SYS_EVENT_SINGLE_BLOCK_ON);
    EXPECT_TRUE(rt_get_sys_state(SYS_IS_SINGLE_BLOCK));
    EXPECT_EQ(cycle_tp->trigger_event, SINGLE_BLOCK_ENABLE);

    start_with_block(make_motion_block(LINE, PMC_MODE_CHOP, 14, 10.0));
    ASSERT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->plan_block->start_vel, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->plan_block->end_vel, 0.0));

    run_until_state(EXEC_PAUSING);

    rt_data->nc_event = SYS_EVENT_NONE;
    handle_auto_cmd();

    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_EQ(cycle_tp->running_stage, PROGRAM_WAIT);
    int is_moving = 1;
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    EXPECT_EQ(is_moving, 0);

    send_event(SYS_EVENT_RERUN);
    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
}

TEST_F(TestAuto, MachineLockSuppressesMotionUntilDisabled)
{
    start_with_block(make_motion_block(LINE, PMC_MODE_CHOP, 15, 100.0));
    ASSERT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    ASSERT_DOUBLE_EQ(cycle_tp->pos_cmd_data.target_vel, 60.0);

    send_event(SYS_EVENT_MAC_LOCK_ON);

    EXPECT_TRUE(rt_get_sys_state(SYS_IS_MAC_LOCK_ON));
    EXPECT_EQ(cycle_tp->trigger_event, MAC_LOCK_ENABLE);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_DOUBLE_EQ(cycle_tp->pos_cmd_data.target_vel, 0.0);
    EXPECT_FALSE(queue_is_empty(plan_queue));

    for (int i = 0; i < 2000 && !rt_get_sys_state(SYS_IS_MAC_LOCK_READY); ++i) {
        handle_auto_cmd();
    }
    ASSERT_TRUE(rt_get_sys_state(SYS_IS_MAC_LOCK_READY));

    send_event(SYS_EVENT_MAC_LOCK_OFF);

    EXPECT_FALSE(rt_get_sys_state(SYS_IS_MAC_LOCK_ON));
    EXPECT_EQ(cycle_tp->trigger_event, NONE_EVENT);
    EXPECT_FALSE(rt_get_sys_state(SYS_IS_MAC_LOCK_READY));

    handle_auto_cmd();
    EXPECT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_DOUBLE_EQ(cycle_tp->pos_cmd_data.target_vel, 60.0);
}

TEST_F(TestAuto, AuxiliaryFunctionLockSkipsCustomMst)
{
    rt_set_sys_state(SYS_IS_MSBT_LOCK, true);
    start_with_block(make_mst_block(16, M03, 1, 120));

    EXPECT_EQ(cycle_tp->exec_cmd, DO_NULL_CMD);
    EXPECT_EQ(cycle_tp->mstb_cmd_data.mcode_size, 0);
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_FALSE(read_reg_bit(REG_F, 1, 3));
    EXPECT_FALSE(read_reg_bit(REG_F, 7, 0));
    EXPECT_TRUE(queue_is_empty(plan_queue));
}

TEST_F(TestAuto, BlockSkipEventsUpdateSystemState)
{
    send_event(SYS_EVENT_BLOCK_SKIP_ON);
    EXPECT_TRUE(rt_get_sys_state(SYS_IS_BLOCK_SKIP));
    EXPECT_EQ(cycle_tp->trigger_event, BLOCK_SKIP_ENABLE);

    send_event(SYS_EVENT_BLOCK_SKIP_OFF);
    EXPECT_FALSE(rt_get_sys_state(SYS_IS_BLOCK_SKIP));
    EXPECT_EQ(cycle_tp->trigger_event, NONE_EVENT);
}

TEST_F(TestAuto, FeedOverrideReplansFeedBlock)
{
    PLAN_BLOCK block = make_motion_block(LINE, PMC_MODE_CHOP, 20, 100.0);
    start_with_block(block);

    setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, 50.0, 0);
    rt_data->nc_event = SYS_EVENT_FEED_OVERRIDE;
    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 30.0));
}

TEST_F(TestAuto, RapidOverrideReplansRapidBlock)
{
    PLAN_BLOCK block = make_motion_block(RAPID, PMC_MODE_CHOP, 21, 100.0);
    start_with_block(block);

    setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, 25.0, 0);
    rt_data->nc_event = SYS_EVENT_RAPID_OVERRIDE;
    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_MOVE_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 25.0));
}

TEST_F(TestAuto, FeedOverrideDoesNotReplanRapidBlock)
{
    start_with_block(make_motion_block(RAPID, PMC_MODE_CHOP, 22, 100.0));
    ASSERT_EQ(cycle_tp->exec_cmd, DO_MOVE_CMD);

    setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, 25.0, 0);
    send_event(SYS_EVENT_FEED_OVERRIDE);

    EXPECT_EQ(cycle_tp->exec_cmd, DO_MOVE_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 100.0));
}

TEST_F(TestAuto, RapidOverrideDoesNotReplanFeedBlock)
{
    start_with_block(make_motion_block(LINE, PMC_MODE_CHOP, 23, 100.0));
    ASSERT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);

    setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, 25.0, 0);
    send_event(SYS_EVENT_RAPID_OVERRIDE);

    EXPECT_EQ(cycle_tp->exec_cmd, DO_CHOP_CMD);
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 60.0));
}

TEST_F(TestAuto, TapIgnoresPauseAndFeedOverride)
{
    PLAN_BLOCK block = make_motion_block(RIGID_TAP, PMC_MODE_TAP, 30, 100.0);
    start_with_block(block);
    ASSERT_EQ(cycle_tp->exec_cmd, DO_TAP_CMD);

    double target_before = cycle_tp->pos_cmd_data.target_vel;

    rt_data->nc_event = SYS_EVENT_PAUSE;
    handle_auto_cmd();
    EXPECT_EQ(cycle_tp->exec_state, EXEC_RUNNING);
    EXPECT_EQ(cycle_tp->exec_cmd, DO_TAP_CMD);

    setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, 10.0, 0);
    rt_data->nc_event = SYS_EVENT_FEED_OVERRIDE;
    handle_auto_cmd();
    EXPECT_TRUE(IsEqualTo(cycle_tp->pos_cmd_data.target_vel, target_before));
}

TEST_F(TestAuto, M03OutputsMAndSpindleDataAndCompletesOnFin)
{
    start_with_block(make_mst_block(31, M03, 1, 120));

    ASSERT_EQ(cycle_tp->exec_cmd, DO_MSTB_CMD);
    EXPECT_EQ(read_reg8(REG_F, 10), 3);
    EXPECT_EQ(read_reg8(REG_F, 22), 120);
    EXPECT_TRUE(read_reg_bit(REG_F, 1, 3));
    EXPECT_TRUE(read_reg_bit(REG_F, 7, 0));
    EXPECT_EQ(var_chan[0][VAR_CHAN_FIN_SIGNAL].value.i, 1);

    finish_current_mst();

    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_FALSE(read_reg_bit(REG_F, 1, 3));
    EXPECT_FALSE(read_reg_bit(REG_F, 7, 0));
    EXPECT_EQ(var_chan[0][VAR_CHAN_FIN_SIGNAL].value.i, 0);
}

TEST_F(TestAuto, M06OutputsToolDataAndCompletesOnFin)
{
    start_with_block(make_mst_block(32, M06, 1, 0, 7));

    ASSERT_EQ(cycle_tp->exec_cmd, DO_MSTB_CMD);
    EXPECT_EQ(read_reg8(REG_F, 10), 6);
    EXPECT_EQ(read_reg8(REG_F, 26), 7);
    EXPECT_TRUE(read_reg_bit(REG_F, 7, 3));

    finish_current_mst();

    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_FALSE(read_reg_bit(REG_F, 7, 3));
}

TEST_F(TestAuto, M30FinEndsProgramAndIncrementsPartCounter)
{
    setVarChanInt(0, VAR_CHAN_PART_CNTR, 4, 0);
    start_with_block(make_mst_block(33, M30));
    rt_set_chan_state(0, CHAN_IS_PARSERING, false);

    ASSERT_EQ(cycle_tp->exec_cmd, DO_MSTB_CMD);
    EXPECT_EQ(read_reg8(REG_F, 10), 0x1e);
    EXPECT_TRUE(read_reg_bit(REG_F, 9, 4));

    finish_current_mst();

    EXPECT_EQ(cycle_tp->running_stage, PROGRAM_END);
    EXPECT_EQ(var_chan[0][VAR_CHAN_PART_CNTR].value.i, 5);
    int run_row = -1;
    getVarChanInt(0, VAR_CHAN_RUN_ROW, &run_row, 0);
    EXPECT_EQ(run_row, 0);

    rt_data->nc_event = SYS_EVENT_NONE;
    handle_auto_cmd();
    EXPECT_EQ(cycle_tp->exec_state, EXEC_IDLE);
}

TEST_F(TestAuto, EmptyQueuesAndStoppedParserEndProgram)
{
    cycle_tp->exec_state = EXEC_RUNNING;
    cycle_tp->running_stage = PROGRAM_WAIT;
    rt_data->chan_state[0][CHAN_IS_PARSERING] = false;
    rt_data->path_data.parser_error_no = -1;

    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->running_stage, PROGRAM_END);

    rt_data->nc_event = SYS_EVENT_NONE;
    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->exec_state, EXEC_IDLE);

    int is_running = 1;
    int is_moving = 1;
    getVarChanInt(0, VAR_CHAN_IS_RUNNING, &is_running, 0);
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    EXPECT_EQ(is_running, 0);
    EXPECT_EQ(is_moving, 0);
}
