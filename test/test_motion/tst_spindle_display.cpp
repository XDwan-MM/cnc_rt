#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <memory>

#include "global.h"
#include "test_env.h"
#include <hmisysapi.h>

#define private public
#include "servosamplesampler.h"
#undef private

namespace {
class SpindleDisplayTest : public ::testing::Test {
protected:
    RT_AXIS saved_axes[SYS_AXES_NUM];
    NC_MODE saved_mode;
    VarUnion saved_modal{};
    double saved_zero = 0;
    double saved_z_offset = 0;
    bool saved = false;

    void SetUp() override {
        if (!testEnvReady()) GTEST_SKIP() << testEnvMessage();
        std::memcpy(saved_axes, rt_data->axis_data, sizeof(saved_axes));
        saved_mode = rt_data->current_mode;
        getVarChanValue(0, VAR_CHAN_MODAL, 14, &saved_modal, 0);
        getVarAxisDouble(AXIS_SP, VAR_AXIS_REAL_ZERO_EX, &saved_zero, 0);
        getCrdsValue(CRDS_TYPE_G5X, AXIS_Z, &saved_z_offset);
        saved = true;
        std::memset(rt_data->axis_data, 0, sizeof(saved_axes));
        rt_data->current_mode = MODE_EDIT;
        VarUnion modal{};
        modal.value.i = 1;
        ASSERT_EQ(setVarChanValue(0, VAR_CHAN_MODAL, 14, &modal, 0), 0);
        setVarAxisDouble(AXIS_SP, VAR_AXIS_REAL_ZERO_EX, 2, 0);
        setCrdsValue(CRDS_TYPE_G5X, AXIS_Z, 0);
    }

    void TearDown() override {
        if (!saved) return;
        std::memcpy(rt_data->axis_data, saved_axes, sizeof(saved_axes));
        rt_data->current_mode = saved_mode;
        setVarChanValue(0, VAR_CHAN_MODAL, 14, &saved_modal, 0);
        setVarAxisDouble(AXIS_SP, VAR_AXIS_REAL_ZERO_EX, saved_zero, 0);
        setCrdsValue(CRDS_TYPE_G5X, AXIS_Z, saved_z_offset);
    }

    double displayed(int variable, int axis = AXIS_SP) {
        double value = 0;
        EXPECT_EQ(getVarAxisDouble(axis, variable, &value, 0), 0);
        return value;
    }
};
}

TEST_F(SpindleDisplayTest, OldMirrorsDoNotContaminateSpindleCoordinates) {
    auto &sp = rt_data->axis_data[AXIS_SP];
    sp.wcs_cmd_pos_cur = 1083;
    sp.mac_cmd_pos = 1003;
    sp.motor_cmd_pos = 888;
    sp.motor_act_pos = 1082.5;
    sp.fb_pos_no_comp = -900;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_WCS_EX), 2.5);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX), 2.5);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_RCS_EX), .5);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_ACT_POS_EX), 1082.5);
    EXPECT_DOUBLE_EQ(sp.mac_cmd_pos, 1003);
    EXPECT_DOUBLE_EQ(sp.motor_cmd_pos, 888);
    EXPECT_DOUBLE_EQ(sp.wcs_cmd_pos_cur, 1083);
}

TEST_F(SpindleDisplayTest, RezeroAndNextMoveDoNotReuseOldDisplayOrigin) {
    auto &sp = rt_data->axis_data[AXIS_SP];
    sp.mac_cmd_pos = sp.motor_cmd_pos = 1083;
    sp.wcs_cmd_pos_cur = 0;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX), 0);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_RCS_EX), -2);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_WCS_EX), 0);
    sp.wcs_cmd_pos_cur = 720.5;
    sp.motor_act_pos = 719.75;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX), 359.75);
    EXPECT_DOUBLE_EQ(sp.mac_cmd_pos, 1083);
    sp.wcs_cmd_pos_cur = -.5;
    sp.motor_act_pos = -1.25;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX), -1.25);
}

TEST_F(SpindleDisplayTest, FeedbackAfterResetUpdatesDisplayWithoutChangingTarget) {
    auto &sp = rt_data->axis_data[AXIS_SP];
    sp.wcs_cmd_pos_cur = 100;
    sp.mac_cmd_pos = 100;
    sp.motor_cmd_pos = 100;
    sp.motor_act_pos = 97.25;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_WCS_EX), 97.25);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX), 97.25);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_RCS_EX), 95.25);
    sp.motor_act_pos = 97;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_WCS_EX), 97);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX), 97);
    EXPECT_DOUBLE_EQ(sp.wcs_cmd_pos_cur, 100);
    EXPECT_DOUBLE_EQ(sp.mac_cmd_pos, 100);
    EXPECT_DOUBLE_EQ(sp.motor_cmd_pos, 100);
}

TEST_F(SpindleDisplayTest, SpindleDoesNotInheritZWorkOffset) {
    ASSERT_EQ(setCrdsValue(CRDS_TYPE_G5X, AXIS_Z, 7.5), 0);
    rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur = 20;
    rt_data->axis_data[AXIS_SP].motor_act_pos = 725;
    double spindle_offset = 0;
    if (getCrdsValue(CRDS_TYPE_G5X, AXIS_SP, &spindle_offset) != 0)
        spindle_offset = 0;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_WCS_EX, AXIS_Z), 12.5);
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_WCS_EX), std::fmod(725 - spindle_offset, 360));
}

TEST_F(SpindleDisplayTest, FeedAxisDisplayKeepsExistingSource) {
    auto &x = rt_data->axis_data[AXIS_X];
    x.wcs_cmd_pos_cur = 20;
    x.mac_cmd_pos = 25;
    x.motor_act_pos = 24;
    x.fb_pos_no_comp = 23;
    shm_var_update();
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_CMD_POS_EX, AXIS_X), 25);
#ifdef DEBUG_MODE_ENABLED
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_ACT_POS_EX, AXIS_X), 23);
#else
    EXPECT_DOUBLE_EQ(displayed(VAR_AXIS_ACT_POS_EX, AXIS_X), 24);
#endif
}

TEST_F(SpindleDisplayTest, SamplesUseUnwrappedSpindleTargetAndIndependentFeedback) {
    auto data = std::make_unique<CncServoSampleData>();
    ServoSampleSampler sampler;
    sampler._shared = data.get();
    sampler._config.capacity = 2;
    sampler._config.channels[0].enabled = 1;
    sampler._config.channels[0].signalId = CNC_SERVO_SIGNAL_CMD_SPINDLE_ANGLE;
    sampler._config.channels[1].enabled = 1;
    sampler._config.channels[1].signalId = CNC_SERVO_SIGNAL_ACT_SPINDLE_ANGLE;
    sampler._config.channels[2].enabled = 1;
    sampler._config.channels[2].signalId = CNC_SERVO_SIGNAL_CMD_POS;
    sampler._config.channels[2].axis = AXIS_SP;
    auto &sp = rt_data->axis_data[AXIS_SP];
    sp.wcs_cmd_pos_cur = 1083;
    sp.mac_cmd_pos = sp.motor_cmd_pos = 888;
    sp.motor_act_pos = 1082.5;
    sampler.sampleOne();
    EXPECT_EQ(data->sampleCount, 1U);
    EXPECT_DOUBLE_EQ(data->samples[0].values[0], 1083);
    EXPECT_DOUBLE_EQ(data->samples[0].values[1], 1082.5);
    EXPECT_DOUBLE_EQ(data->samples[0].values[2], 1083);
    EXPECT_DOUBLE_EQ(sp.motor_cmd_pos, 888);
}
