#include <gtest/gtest.h>
#include <cstring>

#include "emcmot.h"
#include "global.h"
#include "rt_data.h"
#include "test_env.h"

namespace
{

class TestEmcmot : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady())
        {
            GTEST_SKIP() << testEnvMessage();
        }

        std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        update_pos_flag = 0;
    }

    void TearDown() override
    {
        if (rt_data != nullptr)
        {
            std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        }
        update_pos_flag = 0;
    }
};

} // namespace

TEST_F(TestEmcmot, RebuildPositionUsesLatestFeedbackWithoutDoubleCompensation)
{
    const int compensation_axis_count = AXIS_SP;
    const double previous_mac_pos[] = {100.0, -25.0, 8.0};
    const double feedback_error[] = {0.015, -0.004, 0.009};
    const double pitch_comp[] = {0.120, -0.080, 0.050};
    const double backlash_comp[] = {0.030, 0.020, -0.010};
    const double straightness1_comp[] = {0.004, -0.003, 0.002};
    const double straightness2_comp[] = {-0.001, 0.006, -0.005};

    for (int axis = 0; axis < compensation_axis_count; axis++)
    {
        RT_AXIS *axis_data;
        double previous_total_comp;
        double current_uncompensated_pos;

        axis_data = &rt_data->axis_data[axis];
        axis_data->pitch_comp_val = pitch_comp[axis];
        axis_data->backlash_comp_val = backlash_comp[axis];
        axis_data->straightness1_comp_val = straightness1_comp[axis];
        axis_data->straightness2_comp_val = straightness2_comp[axis];
        axis_data->total_comp_cal = axis_data->pitch_comp_val + axis_data->backlash_comp_val +
                                    axis_data->straightness1_comp_val + axis_data->straightness2_comp_val;
        previous_total_comp = axis_data->total_comp_cal;
        current_uncompensated_pos = previous_mac_pos[axis] + feedback_error[axis];

        axis_data->motor_act_pos = current_uncompensated_pos + previous_total_comp;
        axis_data->motor_cmd_pos = -1000.0;
        axis_data->fb_pos_no_comp = previous_mac_pos[axis];
    }

    emcmot_test_rebuild_axis_positions_from_feedback();

    EXPECT_EQ(update_pos_flag, 1);
    for (int axis = 0; axis < compensation_axis_count; axis++)
    {
        RT_AXIS *axis_data;
        double previous_total_comp;
        double current_uncompensated_pos;
        double next_motor_cmd_pos;

        axis_data = &rt_data->axis_data[axis];
        previous_total_comp = axis_data->total_comp_cal;
        current_uncompensated_pos = previous_mac_pos[axis] + feedback_error[axis];
        next_motor_cmd_pos = axis_data->mac_cmd_pos + previous_total_comp;

        EXPECT_DOUBLE_EQ(axis_data->wcs_cmd_pos_cur, current_uncompensated_pos);
        EXPECT_DOUBLE_EQ(axis_data->wcs_cmd_pos_pre, current_uncompensated_pos);
        EXPECT_DOUBLE_EQ(axis_data->mac_cmd_pos, current_uncompensated_pos);
        EXPECT_DOUBLE_EQ(axis_data->fb_pos_no_comp, current_uncompensated_pos);
        EXPECT_DOUBLE_EQ(axis_data->motor_cmd_pos, axis_data->motor_act_pos);
        EXPECT_DOUBLE_EQ(next_motor_cmd_pos, axis_data->motor_act_pos);
        EXPECT_NE(next_motor_cmd_pos, axis_data->motor_act_pos + previous_total_comp);
    }
}
