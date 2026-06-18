#include <gtest/gtest.h>

#include "test_env.h"

#include "datatype.h"
#include "global.h"
#include "homing.h"
#include <varapi.h>
#include <parapi.h>

/********************************************************************
 * @file        tst_homing.cpp
 * @brief       src/motion/homing 回零模块单元测试
********************************************************************/

class TestHoming : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady()) {
            GTEST_SKIP() << testEnvMessage();
        }
    }

    // 写入一组轴参数
    void set_axis_param(int axis, int index, double real_value)
    {
        ParamUnit p = {};
        p.value.val_real = real_value;
        setParData(PARAMAN_FILE_AXIS, axis, index, p);
    }
    void set_axis_param_uint(int axis, int index, unsigned int uint_value)
    {
        ParamUnit p = {};
        p.value.val_uint = uint_value;
        setParData(PARAMAN_FILE_AXIS, axis, index, p);
    }
};

// homing_tp_init 应为每个进给轴分配 homing_tp，并设置默认状态
TEST_F(TestHoming, TpInitAllocates)
{
    homing_tp_init();
    ASSERT_NE(homing_tp, nullptr);
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        ASSERT_NE(homing_tp[i].homing_block, nullptr);
        ASSERT_NE(homing_tp[i].homing_profile, nullptr);
        ASSERT_NE(homing_tp[i].traj_data, nullptr);
        // 默认空闲、正向、直线块
        EXPECT_EQ(homing_tp[i].homing_state, ABS_EXEC_IDLE);
        EXPECT_EQ(homing_tp[i].homing_direction, AXIS_HOMING_POSITIVE);
        EXPECT_EQ(homing_tp[i].homing_block->type, LINE);
    }
}

// homing_tp_init 应从 par_axis 读取快移速度/加速度/捷度写入 homing_block
TEST_F(TestHoming, TpInitReadsAxisParams)
{
    const double rapid_spd = 1000.0;
    const double rapid_acc = 5000.0;
    const double rapid_jk  = 20000.0;
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        set_axis_param(i, PAR_AX_RAPID_SPD, rapid_spd);
        set_axis_param(i, PAR_AX_RAPID_ACC, rapid_acc);
        set_axis_param(i, PAR_AX_RAPID_JK,  rapid_jk);
    }

    homing_tp_init();

    for (int i = 0; i < SYS_AXES_NUM; i++) {
        ASSERT_NE(homing_tp[i].homing_block, nullptr);
        EXPECT_TRUE(IsEqualTo(homing_tp[i].homing_block->max_vel,  rapid_spd));
        EXPECT_TRUE(IsEqualTo(homing_tp[i].homing_block->feed_vel, rapid_spd));
        EXPECT_TRUE(IsEqualTo(homing_tp[i].homing_block->max_acc,  rapid_acc));
        EXPECT_TRUE(IsEqualTo(homing_tp[i].homing_block->max_jerk, rapid_jk));
        EXPECT_TRUE(IsEqualTo(homing_tp[i].homing_block->end_vel,  0.0));
    }
}

// homing_tp_reset 应将状态置为 IDLE、清回零请求、清空 traj_data
TEST_F(TestHoming, TpResetClearsState)
{
    homing_tp_init();
    ASSERT_NE(homing_tp, nullptr);
    // 人为制造一个“回零进行中”的脏状态
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        ASSERT_NE(homing_tp[i].traj_data, nullptr);
        homing_tp[i].homing_state = ABS_EXEC_HOMING;
        homing_tp[i].is_ref_on = true;
        homing_tp[i].traj_data->v = 123.0;
        homing_tp[i].traj_data->a = 456.0;
    }

    homing_tp_reset();

    for (int i = 0; i < SYS_AXES_NUM; i++) {
        ASSERT_NE(homing_tp[i].traj_data, nullptr);
        EXPECT_EQ(homing_tp[i].homing_state, ABS_EXEC_IDLE);
        EXPECT_FALSE(homing_tp[i].is_ref_on);
        EXPECT_TRUE(IsEqualTo(homing_tp[i].traj_data->v, 0.0));
        EXPECT_TRUE(IsEqualTo(homing_tp[i].traj_data->a, 0.0));
    }
}

// homing_handle_estop 应复位 homing_tp 并清除通道“回零中/移动中”标志
TEST_F(TestHoming, HandleEstopClearsFlags)
{
    homing_tp_init();
    ASSERT_NE(homing_tp, nullptr);
    // 预置“回零中/移动中”标志
    setVarChanInt(0, VAR_CHAN_IS_HOMING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        homing_tp[i].homing_state = ABS_EXEC_HOMING;
        homing_tp[i].is_ref_on = true;
    }

    homing_handle_estop();

    int is_homing = -1, is_moving = -1;
    getVarChanInt(0, VAR_CHAN_IS_HOMING, &is_homing, 0);
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    EXPECT_EQ(is_homing, 0);
    EXPECT_EQ(is_moving, 0);
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        EXPECT_EQ(homing_tp[i].homing_state, ABS_EXEC_IDLE);
        EXPECT_FALSE(homing_tp[i].is_ref_on);
    }
}
