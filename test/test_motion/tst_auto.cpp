#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <cstring>
#include "test_env.h"
#include "datatype.h"
#include "global.h"
#include "gcode_queue.h"
#include "mem.h"
#include "mem_gcode.h"
#include "mem_mst.h"
#include "pathplan.h"
#include <parapi.h>
#include <varapi.h>

/********************************************************************
 * @file        tst_auto.cpp
 * @brief       src/motion/auto 自动模式单元测试
 ********************************************************************/

class CycleBlockAxisArriveCheckMock
{
public:
    MOCK_METHOD(void, recordCall, (), ());
};

static void mock_cycle_block_axis_arrive_check(void* ctx)
{
    static_cast<CycleBlockAxisArriveCheckMock*>(ctx)->recordCall();
}

class CycleBlockAxisArriveCheckHookGuard
{
public:
    explicit CycleBlockAxisArriveCheckHookGuard(CycleBlockAxisArriveCheckMock* mock)
    {
        cycle_block_axis_arrive_check_set_hook(mock_cycle_block_axis_arrive_check, mock);
    }

    ~CycleBlockAxisArriveCheckHookGuard()
    {
        cycle_block_axis_arrive_check_set_hook(nullptr, nullptr);
    }
};

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

        for (int i = 0; i < SYS_AXES_NUM; i++) {
            set_axis_param_real(i, PAR_AX_RAPID_SPD, 100.0);
            set_axis_param_real(i, PAR_AX_RAPID_ACC, 1000.0);
            set_axis_param_real(i, PAR_AX_RAPID_JK, 10000.0);
            set_axis_param_real(i, PAR_AX_FEED_SPD, 80.0);
            set_axis_param_real(i, PAR_AX_FEED_ACC, 800.0);
            set_axis_param_real(i, PAR_AX_FEED_JK, 8000.0);
            set_axis_param_real(i, PAR_AX_POS_TOL, 0.01);
            set_axis_param_real(i, PAR_AX_PLMT, 1000.0);
            set_axis_param_real(i, PAR_AX_NLMT, -1000.0);
        }
    }

    static PLAN_BLOCK make_line_block(INTERP_TYPE type, int line_number, double end_x)
    {
        PLAN_BLOCK block = {};
        block.type = type;
        block.mode = PMC_MODE_CHOP;
        block.line_number = line_number;
        block.feed_vel = 30.0;
        block.max_vel = 30.0;
        block.max_acc = 800.0;
        block.max_jerk = 8000.0;
        block.start_pos = (POS){.x = 0.0, .y = 0.0, .z = 0.0};
        block.end_pos = (POS){.x = end_x, .y = 0.0, .z = 0.0};
        block_cal_vector(&block);
        block_cal_length(&block);
        return block;
    }

    void reset_auto_fixture()
    {
        cycle_tp_reset();
        queue_reset(plan_queue);
        queue_reset(pretreat_queue);
        gcode_queue_reset(gcode_queue);
        mcode_queue_reset(mcode_queue);

        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        std::memset(rt_data->chan_state, 0, sizeof(rt_data->chan_state));
        std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        std::memset(rt_data->chan_data, 0, sizeof(rt_data->chan_data));
        rt_data->nc_event = SYS_EVENT_NONE;
        rt_data->target_mode = MODE_MEM;
        rt_data->current_mode = MODE_MEM;
        rt_data->parser_error.parser_error_no = -1;
        rt_data->parser_error.parser_error_line = 0;
        rt_data->chan_state[0][CHAN_IS_GCODE_END] = true;
        rt_data->chan_state[0][CHAN_IS_MCODE_END] = true;

        if (mcode_data != nullptr) {
            std::memset(mcode_data, 0, sizeof(MCODE_DATA));
            mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
            mcode_data->pmc_end = true;
        }

        setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 0.0, 0);
        setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, 100.0, 0);
        setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, 100.0, 0);
        setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_PROGEND, 0, 0);
        setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);

        for (int i = 0; i < SYS_AXES_NUM; i++) {
            setVarAxisDouble(i, VAR_AXIS_CMD_VEL, 0.0, 0);
        }
    }
};

// G00程序段cycle_tp_is_finished 后能执行cycle_block_axis_arrive_check一次
TEST_F(TestAuto, AutoExecuteG00ArriveCheck)
{
    CycleBlockAxisArriveCheckMock mock;
    CycleBlockAxisArriveCheckHookGuard hook_guard(&mock);
    EXPECT_CALL(mock, recordCall()).Times(1);

    queue_reset(plan_queue);

    PLAN_BLOCK block = make_line_block(RAPID, 40, 10.0);
    ASSERT_EQ(queue_push(plan_queue, &block), GNC_OPERATOR_SUCCESS);
    *cycle_tp->plan_block = block;

    cycle_tp->mem_exec_state = EXEC_RUNNING;
    cycle_tp->is_event_trigger = false;
    cycle_tp->is_arrive_at_the_end = true;
    cycle_tp->interp_cycle = 2;
    cycle_tp->vel_profile->t_cycle = 1.0;

    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = block.end_pos.x;
    rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur = block.end_pos.y;
    rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur = block.end_pos.z;
    rt_data->axis_data[AXIS_X].motor_act_pos = block.end_pos.x;
    rt_data->axis_data[AXIS_Y].motor_act_pos = block.end_pos.y;
    rt_data->axis_data[AXIS_Z].motor_act_pos = block.end_pos.z;
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 60.0, 0);

    auto_execute_gcode();

    EXPECT_EQ(cycle_tp->plan_block->end_pos.x, 10.0);
    EXPECT_EQ(rt_data->axis_data[AXIS_X].motor_act_pos, 10.0);

    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_FALSE(cycle_tp->is_arrive_at_the_end);

    queue_reset(plan_queue);
}

// plan_block_get_next 弹出最后一段后队列为空，下一周期仍保持“未到段末”，避免继续取下一段。
TEST_F(TestAuto, PlanBlockEmptyKeepsNotArrivedNextCycle)
{
    queue_reset(plan_queue);

    PLAN_BLOCK block = make_line_block(LINE, 30, 10.0);
    ASSERT_EQ(queue_push(plan_queue, &block), GNC_OPERATOR_SUCCESS);
    *cycle_tp->plan_block = block;
    cycle_tp->mem_exec_state = EXEC_RUNNING;
    cycle_tp->is_event_trigger = false;
    cycle_tp->is_arrive_at_the_end = true;
    cycle_tp->interp_cycle = 2;
    cycle_tp->vel_profile->t_cycle = 1.0;
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 60.0, 0);

    auto_execute_gcode();

    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_FALSE(cycle_tp->is_arrive_at_the_end);
    EXPECT_FALSE(cycle_tp->is_event_trigger);

    auto_execute_gcode();

    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_FALSE(cycle_tp->is_arrive_at_the_end);
    EXPECT_FALSE(cycle_tp->is_event_trigger);

    queue_reset(plan_queue);
}

// 上周期 plan_block_get_next 返回空后，下周期有新数据时应走 plan_block_get_update，而不是再次弹出该段。
TEST_F(TestAuto, PlanBlockEmptyThenNotEmptyKeepsUpdateNextCycle)
{
    queue_reset(plan_queue);

    PLAN_BLOCK first_block = make_line_block(LINE, 30, 10.0);
    ASSERT_EQ(queue_push(plan_queue, &first_block), GNC_OPERATOR_SUCCESS);
    *cycle_tp->plan_block = first_block;
    cycle_tp->mem_exec_state = EXEC_RUNNING;
    cycle_tp->is_event_trigger = false;
    cycle_tp->is_arrive_at_the_end = true;
    cycle_tp->interp_cycle = 2;
    cycle_tp->vel_profile->t_cycle = 1.0;
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 60.0, 0);

    auto_execute_gcode();

    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_FALSE(cycle_tp->is_arrive_at_the_end);

    PLAN_BLOCK next_block = make_line_block(LINE, 31, 20.0);
    ASSERT_EQ(queue_push(pretreat_queue, &next_block), GNC_OPERATOR_SUCCESS);

    auto_execute_gcode();

    EXPECT_EQ(queue_get_size(pretreat_queue), 0);
    EXPECT_EQ(queue_get_size(plan_queue), 1);
    ASSERT_NE(queue_peak(plan_queue), nullptr);
    EXPECT_EQ(queue_peak(plan_queue)->line_number, 31);
    EXPECT_EQ(cycle_tp->plan_block->line_number, 31);
    EXPECT_TRUE(cycle_tp->is_arrive_at_the_end);

    queue_reset(plan_queue);
}


// 初始化时
TEST_F(TestAuto, CycleTpInitAllocatesDefaults)
{
    ASSERT_NE(cycle_tp, nullptr);
    ASSERT_NE(cycle_tp->plan_block, nullptr);
    ASSERT_NE(cycle_tp->vel_profile, nullptr);
    ASSERT_NE(cycle_tp->traj_data, nullptr);
    EXPECT_EQ(cycle_tp->mem_exec_state, EXEC_IDLE);
    EXPECT_EQ(cycle_tp->plan_block->type, UNKNOWN);
    EXPECT_EQ(cycle_tp->interp_cycle, 1);
    EXPECT_TRUE(cycle_tp->is_paused_inside_block);
    EXPECT_FALSE(cycle_tp->is_event_trigger);
}

// 复位时
TEST_F(TestAuto, CycleTpResetClearsMotionState)
{
    cycle_tp->mem_exec_state = EXEC_RUNNING;
    cycle_tp->plan_block->type = LINE;
    cycle_tp->traj_data->s = 12.0;
    cycle_tp->traj_data->v = 5.0;
    cycle_tp->is_event_trigger = true;
    cycle_tp->is_arrive_at_the_end = true;
    cycle_tp->is_resetting = true;
    cycle_tp->interp_cycle = 99;
    cycle_tp->vel_profile->t_cycle = 20.0;
    cycle_tp->current_distance = 1.0;
    cycle_tp->target_distance = 2.0;
    cycle_tp->accum_distance = 3.0;
    cycle_tp->remain_distance = 4.0;

    cycle_tp_reset();

    EXPECT_EQ(cycle_tp->mem_exec_state, EXEC_IDLE);
    EXPECT_EQ(cycle_tp->plan_block->type, UNKNOWN);
    EXPECT_TRUE(IsEqualTo(cycle_tp->traj_data->s, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->traj_data->v, 0.0));
    EXPECT_FALSE(cycle_tp->is_event_trigger);
    EXPECT_TRUE(cycle_tp->is_paused_inside_block);
    EXPECT_FALSE(cycle_tp->is_arrive_at_the_end);
    EXPECT_FALSE(cycle_tp->is_resetting);
    EXPECT_EQ(cycle_tp->interp_cycle, 1);
    EXPECT_TRUE(IsEqualTo(cycle_tp->vel_profile->t_cycle, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->current_distance, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->target_distance, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->accum_distance, 0.0));
    EXPECT_TRUE(IsEqualTo(cycle_tp->remain_distance, 0.0));
}

TEST_F(TestAuto, HandleAutoCmdRerunSetsRunningState)
{
    rt_data->nc_event = SYS_EVENT_RERUN;

    handle_auto_cmd();

    EXPECT_EQ(cycle_tp->mem_exec_state, EXEC_RUNNING);
    EXPECT_TRUE(cycle_tp->is_event_trigger);
    EXPECT_EQ(mcode_data->mcode_state, MCODE_EXECUTE_RUNNING);
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

TEST_F(TestAuto, GcodeHandleResetClearsQueuesAndFlags)
{
    queue_reset(plan_queue);

    PLAN_BLOCK block = make_line_block(RAPID, 10, 5.0);
    GCODE_BLOCK gcode = {};
    gcode.type = LINE;
    gcode.line_number = 10;
    ASSERT_EQ(queue_push(plan_queue, &block), GNC_OPERATOR_SUCCESS);
    ASSERT_EQ(queue_push(pretreat_queue, &block), GNC_OPERATOR_SUCCESS);
    ASSERT_EQ(gcode_queue_push(gcode_queue, &gcode), GNC_OPERATOR_SUCCESS);
    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_PROGEND, 0, 0);
    rt_data->chan_state[0][CHAN_IS_PARSERING] = true;
    rt_data->chan_state[0][CHAN_IS_GCODE_END] = false;
    cycle_tp->mem_exec_state = EXEC_RUNNING;

    gcode_handle_reset();

    EXPECT_TRUE(queue_is_empty(plan_queue));
    EXPECT_TRUE(queue_is_empty(pretreat_queue));
    EXPECT_TRUE(gcode_queue_is_empty(gcode_queue));
    EXPECT_EQ(cycle_tp->mem_exec_state, EXEC_IDLE);
    EXPECT_EQ(cycle_tp->plan_block->type, UNKNOWN);
    EXPECT_FALSE(rt_get_chan_state(0, CHAN_IS_PARSERING));
    EXPECT_TRUE(rt_get_chan_state(0, CHAN_IS_GCODE_END));

    int is_running = 0;
    int is_moving = 0;
    int is_progend = 0;
    getVarChanInt(0, VAR_CHAN_IS_RUNNING, &is_running, 0);
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    getVarChanInt(0, VAR_CHAN_IS_PROGEND, &is_progend, 0);
    EXPECT_EQ(is_running, 0);
    EXPECT_EQ(is_moving, 0);
    EXPECT_EQ(is_progend, 1);

    queue_reset(plan_queue);
}
