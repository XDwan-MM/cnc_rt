#include <gtest/gtest.h>
#include <cstring>
#include "comp_data.h"
#include "errorcompensationapi.h"
#include "global.h"
#include "test_env.h"

namespace
{

constexpr int STUB_DATA_CAPACITY = 2048;
constexpr int STUB_MAX_CALLS = 16;

struct StraightnessReadCall
{
    int start;
    int count;
};

int straightness_data[STUB_DATA_CAPACITY];
StraightnessReadCall straightness_read_calls[STUB_MAX_CALLS];
int straightness_read_call_count;
int straightness_read_result;

void reset_straightness_reader()
{
    std::memset(straightness_data, 0, sizeof(straightness_data));
    std::memset(straightness_read_calls, 0, sizeof(straightness_read_calls));
    straightness_read_call_count = 0;
    straightness_read_result = 0;
}

void seed_straightness_group(CompParams *params,
                             int group,
                             int type,
                             double start_pos,
                             int point_count,
                             double step,
                             int table_start)
{
    if (group == 1)
    {
        params->stra1_type = type;
        params->stra1_start_pos = start_pos;
        params->stra1_point_cnt = point_count;
        params->stra1_step = step;
        params->stra1_table_start_idx = table_start;
    }
    else
    {
        params->stra2_type = type;
        params->stra2_start_pos = start_pos;
        params->stra2_point_cnt = point_count;
        params->stra2_step = step;
        params->stra2_table_start_idx = table_start;
    }
}

void seed_acmp_straightness_group(int axis,
                                  int group,
                                  int type,
                                  double start_pos,
                                  int point_count,
                                  double step,
                                  int table_start)
{
    if (group == 1)
    {
        par_acmp[axis][PAR_CMP_STRA1_TYPE].value.val_int = type;
        par_acmp[axis][PAR_CMP_STRA1_START].value.val_real = start_pos;
        par_acmp[axis][PAR_CMP_STRA1_NUM].value.val_int = point_count;
        par_acmp[axis][PAR_CMP_STRA1_STEP].value.val_real = step;
        par_acmp[axis][PAR_CMP_STRA1_TABLE].value.val_int = table_start;
    }
    else
    {
        par_acmp[axis][PAR_CMP_STRA2_TYPE].value.val_int = type;
        par_acmp[axis][PAR_CMP_STRA2_START].value.val_real = start_pos;
        par_acmp[axis][PAR_CMP_STRA2_NUM].value.val_int = point_count;
        par_acmp[axis][PAR_CMP_STRA2_STEP].value.val_real = step;
        par_acmp[axis][PAR_CMP_STRA2_TABLE].value.val_int = table_start;
    }
}

class TestCompData : public ::testing::Test
{
protected:
    void SetUp() override
    {
        reset_straightness_reader();
        std::memset(par_comp_lib, 0, sizeof(par_comp_lib));

        for (int axis = 0; axis < 3; axis++)
        {
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            straightness_comp_seqlist_init(&straightness1_comp_table[axis]);
            straightness_comp_seqlist_init(&straightness2_comp_table[axis]);
            pitch_comp_table_status[axis] = OP_RES_OK;
            straightness1_comp_table_status[axis] = OP_RES_OK;
            straightness2_comp_table_status[axis] = OP_RES_OK;
        }
    }
};

} // namespace

extern "C" int comp_data_test_get_straightness_data(
    int start, int point_count, int *fwd_comp_arr_um, int *rev_comp_arr_um)
{
    if (straightness_read_call_count < STUB_MAX_CALLS)
    {
        straightness_read_calls[straightness_read_call_count].start = start;
        straightness_read_calls[straightness_read_call_count].count = point_count * 2;
    }
    straightness_read_call_count++;

    if (straightness_read_result != 0)
        return straightness_read_result;
    if (fwd_comp_arr_um == nullptr || rev_comp_arr_um == nullptr || start < 0 || point_count < 0 ||
        start + point_count * 2 > STUB_DATA_CAPACITY)
        return -1;

    std::memcpy(fwd_comp_arr_um, straightness_data + start, sizeof(int) * point_count);
    std::memcpy(rev_comp_arr_um, straightness_data + start + point_count, sizeof(int) * point_count);
    return 0;
}

TEST_F(TestCompData, BuildsBothGroupsFromIndependentContiguousDataRanges)
{
    seed_straightness_group(&par_comp_lib[0], 1, 2, 1.0, 3, 2.0, 10);
    seed_straightness_group(&par_comp_lib[0], 2, 1, -5.0, 2, 0.5, 100);
    seed_straightness_group(&par_comp_lib[1], 1, 1, 20.0, 2, 10.0, 200);

    int axis0_group1[] = {1, 2, 3, -1, -2, -3};
    int axis0_group2[] = {11, 12, -11, -12};
    int axis1_group1[] = {21, 22, -21, -22};
    std::memcpy(straightness_data + 10, axis0_group1, sizeof(axis0_group1));
    std::memcpy(straightness_data + 100, axis0_group2, sizeof(axis0_group2));
    std::memcpy(straightness_data + 200, axis1_group1, sizeof(axis1_group1));

    establish_straightness_comp_table();

    ASSERT_EQ(straightness_read_call_count, 3);
    EXPECT_EQ(straightness_read_calls[0].start, 10);
    EXPECT_EQ(straightness_read_calls[0].count, 6);
    EXPECT_EQ(straightness_read_calls[1].start, 100);
    EXPECT_EQ(straightness_read_calls[1].count, 4);
    EXPECT_EQ(straightness_read_calls[2].start, 200);
    EXPECT_EQ(straightness_read_calls[2].count, 4);

    ASSERT_EQ(straightness1_comp_table[0].length, 3);
    EXPECT_EQ(straightness1_comp_table[0].data[1].moving_ax_pos, 1.0);
    EXPECT_EQ(straightness1_comp_table[0].data[2].moving_ax_pos, 3.0);
    EXPECT_EQ(straightness1_comp_table[0].data[3].moving_ax_pos, 5.0);
    EXPECT_EQ(straightness1_comp_table[0].data[2].fwd_comp_val_um, 2);
    EXPECT_EQ(straightness1_comp_table[0].data[2].rev_comp_val_um, -2);

    ASSERT_EQ(straightness2_comp_table[0].length, 2);
    EXPECT_EQ(straightness2_comp_table[0].data[2].moving_ax_pos, -4.5);
    EXPECT_EQ(straightness2_comp_table[0].data[2].fwd_comp_val_um, 12);
    EXPECT_EQ(straightness2_comp_table[0].data[2].rev_comp_val_um, -12);

    ASSERT_EQ(straightness1_comp_table[1].length, 2);
    EXPECT_EQ(straightness1_comp_table[1].data[2].moving_ax_pos, 30.0);
    EXPECT_EQ(straightness1_comp_table_status[0], OP_RES_OK);
    EXPECT_EQ(straightness2_comp_table_status[0], OP_RES_OK);
    EXPECT_EQ(straightness1_comp_table_status[1], OP_RES_OK);
}

TEST_F(TestCompData, DisabledAndInvalidGroupsAreClearedWithoutReading)
{
    for (int axis = 0; axis < 3; axis++)
    {
        straightness1_comp_table[axis].length = 1;
        straightness2_comp_table[axis].length = 1;
    }

    seed_straightness_group(&par_comp_lib[0], 1, 0, 0.0, 2, 1.0, 0);
    seed_straightness_group(&par_comp_lib[0], 2, 1, 0.0, 0, 1.0, 0);
    seed_straightness_group(&par_comp_lib[1], 1, 3, 0.0, 2, 1.0, 0);
    seed_straightness_group(&par_comp_lib[1], 2, 1, 0.0, 2, 0.0, 0);
    seed_straightness_group(&par_comp_lib[2], 1, 1, 0.0, COMP_ARRAY_MAX_SIZE + 1, 1.0, 0);
    seed_straightness_group(&par_comp_lib[2], 2, 1, 0.0, 2, 1.0, -1);

    establish_straightness_comp_table();

    EXPECT_EQ(straightness_read_call_count, 0);
    EXPECT_EQ(straightness1_comp_table_status[0], OP_RES_OK);
    EXPECT_EQ(straightness2_comp_table_status[0], OP_RES_OK);
    EXPECT_EQ(straightness1_comp_table_status[1], OP_RES_ERR_OUT_OF_RANGE);
    EXPECT_EQ(straightness2_comp_table_status[1], OP_RES_ERR_OUT_OF_RANGE);
    EXPECT_EQ(straightness1_comp_table_status[2], OP_RES_ERR_OUT_OF_RANGE);
    EXPECT_EQ(straightness2_comp_table_status[2], OP_RES_ERR_OUT_OF_RANGE);

    for (int axis = 0; axis < 3; axis++)
    {
        EXPECT_EQ(straightness1_comp_table[axis].length, 0);
        EXPECT_EQ(straightness2_comp_table[axis].length, 0);
    }
}

TEST_F(TestCompData, ReadFailureClearsTableAndStoresFailureStatus)
{
    seed_straightness_group(&par_comp_lib[0], 1, 1, 0.0, 2, 1.0, 5);
    straightness1_comp_table[0].length = 2;
    straightness_read_result = -37;

    establish_straightness_comp_table();

    EXPECT_EQ(straightness_read_call_count, 1);
    EXPECT_EQ(straightness1_comp_table[0].length, 0);
    EXPECT_EQ(straightness1_comp_table_status[0], OP_RES_ERR_OUT_OF_RANGE);
}

TEST_F(TestCompData, OversizedPitchTableIsRejectedInsteadOfTruncated)
{
    par_comp_lib[0].pitch_type = 2;
    par_comp_lib[0].pitch_num = COMP_ARRAY_MAX_SIZE + 1;
    par_comp_lib[0].pitch_table = 0;
    pitch_comp_table[0].length = 3;

    establish_pitch_comp_table();

    EXPECT_EQ(pitch_comp_table[0].length, 0);
    EXPECT_EQ(pitch_comp_table_status[0], OP_RES_ERR_OUT_OF_RANGE);
}

TEST_F(TestCompData, OrdinaryCycleDoesNotSynchronizeOrRebuildTables)
{
    seed_straightness_group(&par_comp_lib[0], 1, 1, 0.0, 2, 1.0, 5);
    straightness1_comp_table[0].length = 7;

    comp_handle_event(SYS_EVENT_NONE);

    EXPECT_EQ(straightness_read_call_count, 0);
    EXPECT_EQ(straightness1_comp_table[0].length, 7);
}

TEST_F(TestCompData, ResetRebuildsTablesAndNotifiesAllStates)
{
    if (!testEnvReady())
        GTEST_SKIP() << testEnvMessage();

    for (int axis = 0; axis < 3; axis++)
    {
        std::memset(par_acmp[axis], 0, sizeof(par_acmp[axis]));
        pitch_comp_init(&pitch_comp_state[axis], 0.001);
        backlash_comp_init(&backlash_comp_state[axis]);
        straightness_comp_init(&straightness1_comp_state[axis], 0.001);
        straightness_comp_init(&straightness2_comp_state[axis], 0.001);
        pitch_comp_state[axis].phase = COMP_PHASE_LOCKED;
        backlash_comp_state[axis].phase = COMP_PHASE_LOCKED;
        straightness1_comp_state[axis].phase = COMP_PHASE_LOCKED;
        straightness2_comp_state[axis].phase = COMP_PHASE_LOCKED;
    }

    seed_acmp_straightness_group(0, 1, 1, 5.0, 2, 5.0, 20);
    straightness_data[20] = 10;
    straightness_data[21] = 20;
    straightness_data[22] = -10;
    straightness_data[23] = -20;

    comp_handle_event(SYS_EVENT_HMI_RESET);

    EXPECT_EQ(straightness_read_call_count, 1);
    EXPECT_EQ(straightness1_comp_table[0].length, 2);
    for (int axis = 0; axis < 3; axis++)
    {
        EXPECT_EQ(pitch_comp_state[axis].phase, COMP_PHASE_RAMPING);
        EXPECT_EQ(backlash_comp_state[axis].phase, COMP_PHASE_RAMPING);
        EXPECT_EQ(straightness1_comp_state[axis].phase, COMP_PHASE_RAMPING);
        EXPECT_EQ(straightness2_comp_state[axis].phase, COMP_PHASE_RAMPING);
    }
}

TEST_F(TestCompData, InitInitializesBothStraightnessGroupsForEveryAxis)
{
    if (!testEnvReady())
        GTEST_SKIP() << testEnvMessage();

    for (int axis = 0; axis < 3; axis++)
    {
        std::memset(par_acmp[axis], 0, sizeof(par_acmp[axis]));
        std::memset(&straightness1_comp_state[axis], 0x7f, sizeof(straightness1_comp_state[axis]));
        std::memset(&straightness2_comp_state[axis], 0x7f, sizeof(straightness2_comp_state[axis]));
    }

    comp_data_init();

    EXPECT_EQ(straightness_read_call_count, 0);
    for (int axis = 0; axis < 3; axis++)
    {
        EXPECT_EQ(straightness1_comp_state[axis].current_output, 0.0);
        EXPECT_EQ(straightness2_comp_state[axis].current_output, 0.0);
        EXPECT_EQ(straightness1_comp_state[axis].step_limit, 0.001);
        EXPECT_EQ(straightness2_comp_state[axis].step_limit, 0.001);
        EXPECT_EQ(straightness1_comp_state[axis].phase, COMP_PHASE_IDLE);
        EXPECT_EQ(straightness2_comp_state[axis].phase, COMP_PHASE_IDLE);
        EXPECT_FALSE(straightness1_comp_state[axis].pos_initialized);
        EXPECT_FALSE(straightness2_comp_state[axis].pos_initialized);
    }
}

TEST_F(TestCompData, LoadsSavedCompensationIntoRuntimeAxesAndStates)
{
    if (!testEnvReady())
        GTEST_SKIP() << testEnvMessage();

    ErrorCompData original_data = {};
    ErrorCompData saved_data = {};
    RT_AXIS original_axes[SYS_AXES_NUM] = {};

    if (getErrorCompData(&original_data) != 0)
        GTEST_SKIP() << "共享内存误差补偿数据读取失败";
    std::memcpy(original_axes, rt_data->axis_data, sizeof(original_axes));
    saved_data = original_data;

    for (int axis = 0; axis < SYS_AXES_NUM; axis++)
    {
        saved_data.axisErrorCompSavedData[axis].pitch_comp_val = 0.1 + axis;
        saved_data.axisErrorCompSavedData[axis].backlash_comp_val = -0.2 - axis;
        saved_data.axisErrorCompSavedData[axis].straightness1_comp_val = 0.03 + axis;
        saved_data.axisErrorCompSavedData[axis].straightness2_comp_val = -0.04 - axis;
    }

    if (setErrorCompData(&saved_data) != 0)
        GTEST_SKIP() << "共享内存误差补偿数据写入失败";

    EXPECT_EQ(comp_data_load_saved_compensation(), 0);
    for (int axis = 0; axis < SYS_AXES_NUM; axis++)
    {
        const RT_AXIS &runtime_axis = rt_data->axis_data[axis];
        const AxisErrorCompSavedData &saved_axis = saved_data.axisErrorCompSavedData[axis];
        EXPECT_DOUBLE_EQ(runtime_axis.pitch_comp_val, saved_axis.pitch_comp_val);
        EXPECT_DOUBLE_EQ(runtime_axis.backlash_comp_val, saved_axis.backlash_comp_val);
        EXPECT_DOUBLE_EQ(runtime_axis.straightness1_comp_val, saved_axis.straightness1_comp_val);
        EXPECT_DOUBLE_EQ(runtime_axis.straightness2_comp_val, saved_axis.straightness2_comp_val);
        EXPECT_DOUBLE_EQ(runtime_axis.total_comp_cal,
                         saved_axis.pitch_comp_val + saved_axis.backlash_comp_val +
                             saved_axis.straightness1_comp_val + saved_axis.straightness2_comp_val);
    }
    for (int axis = 0; axis < 3; axis++)
    {
        EXPECT_DOUBLE_EQ(pitch_comp_state[axis].current_output,
                         saved_data.axisErrorCompSavedData[axis].pitch_comp_val);
        EXPECT_DOUBLE_EQ(backlash_comp_state[axis].current_output,
                         saved_data.axisErrorCompSavedData[axis].backlash_comp_val);
        EXPECT_DOUBLE_EQ(straightness1_comp_state[axis].current_output,
                         saved_data.axisErrorCompSavedData[axis].straightness1_comp_val);
        EXPECT_DOUBLE_EQ(straightness2_comp_state[axis].current_output,
                         saved_data.axisErrorCompSavedData[axis].straightness2_comp_val);
    }

    EXPECT_EQ(setErrorCompData(&original_data), 0);
    std::memcpy(rt_data->axis_data, original_axes, sizeof(original_axes));
}

TEST_F(TestCompData, SavesCurrentCompensationAndPreservesUnimplementedFields)
{
    if (!testEnvReady())
        GTEST_SKIP() << testEnvMessage();

    ErrorCompData original_data = {};
    ErrorCompData saved_data = {};
    ErrorCompData result_data = {};
    RT_AXIS original_axes[SYS_AXES_NUM] = {};

    if (getErrorCompData(&original_data) != 0)
        GTEST_SKIP() << "共享内存误差补偿数据读取失败";
    std::memcpy(original_axes, rt_data->axis_data, sizeof(original_axes));
    saved_data = original_data;

    for (int axis = 0; axis < SYS_AXES_NUM; axis++)
    {
        saved_data.axisErrorCompSavedData[axis].squareness1_comp_val = 10.0 + axis;
        saved_data.axisErrorCompSavedData[axis].thermal_comp_val = 20.0 + axis;
        saved_data.axisErrorCompSavedData[axis].space_geometry_comp_val = 30.0 + axis;
        saved_data.axisErrorCompSavedData[axis].angular_comp_val = 40.0 + axis;
        saved_data.axisErrorCompSavedData[axis].quadrant_comp_val = 50.0 + axis;
        rt_data->axis_data[axis].pitch_comp_val = 0.11 + axis;
        rt_data->axis_data[axis].backlash_comp_val = -0.12 - axis;
        rt_data->axis_data[axis].straightness1_comp_val = 0.13 + axis;
        rt_data->axis_data[axis].straightness2_comp_val = -0.14 - axis;
    }

    if (setErrorCompData(&saved_data) != 0)
        GTEST_SKIP() << "共享内存误差补偿数据写入失败";

    EXPECT_EQ(comp_data_save_current_compensation(), 0);
    EXPECT_EQ(getErrorCompData(&result_data), 0);
    for (int axis = 0; axis < SYS_AXES_NUM; axis++)
    {
        const AxisErrorCompSavedData &result_axis = result_data.axisErrorCompSavedData[axis];
        EXPECT_DOUBLE_EQ(result_axis.pitch_comp_val, rt_data->axis_data[axis].pitch_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.backlash_comp_val, rt_data->axis_data[axis].backlash_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.straightness1_comp_val, rt_data->axis_data[axis].straightness1_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.straightness2_comp_val, rt_data->axis_data[axis].straightness2_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.squareness1_comp_val,
                         saved_data.axisErrorCompSavedData[axis].squareness1_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.thermal_comp_val,
                         saved_data.axisErrorCompSavedData[axis].thermal_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.space_geometry_comp_val,
                         saved_data.axisErrorCompSavedData[axis].space_geometry_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.angular_comp_val,
                         saved_data.axisErrorCompSavedData[axis].angular_comp_val);
        EXPECT_DOUBLE_EQ(result_axis.quadrant_comp_val,
                         saved_data.axisErrorCompSavedData[axis].quadrant_comp_val);
    }

    EXPECT_EQ(setErrorCompData(&original_data), 0);
    std::memcpy(rt_data->axis_data, original_axes, sizeof(original_axes));
}
