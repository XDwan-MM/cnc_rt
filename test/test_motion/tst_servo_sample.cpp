#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "global.h"
#include "parapi.h"
#include "servosampleapi.h"
#include "servosamplesampler.h"
#include "test_env.h"
#include "varapi.h"

namespace {

class TestServoSample : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady()) {
            GTEST_SKIP() << testEnvMessage();
        }

        clearMachineState();
        uint32_t command = 0;
        ASSERT_TRUE(cncServoSampleRequestStop(&command));
        ServoSampleSampler::instance().execute();
        ServoSampleSampler::instance().execute();
        ASSERT_TRUE(cncServoSampleRequestUnregister(&command));
        ServoSampleSampler::instance().execute();
    }

    CncServoSampleConfig config() const
    {
        CncServoSampleConfig result{};
        result.axis = 0;
        result.loopType = CNC_SERVO_LOOP_VELOCITY;
        result.requestedPeriodMs = 1;
        result.capacity = CNC_SERVO_SAMPLE_CAPACITY;
        result.channelMask = (1U << 0) | (1U << 1);
        result.channels[0] = {CNC_SERVO_SIGNAL_CMD_VEL, CNC_SERVO_UNIT_MM_PER_MIN, 1};
        result.channels[1] = {CNC_SERVO_SIGNAL_ACT_VEL, CNC_SERVO_UNIT_MM_PER_MIN, 1};
        return result;
    }

    CncServoSampleConfig compositeConfig() const
    {
        CncServoSampleConfig result = config();
        result.axis = 0;
        result.channels[0] = {CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL,
                              CNC_SERVO_UNIT_MM_PER_MIN, 1};
        result.channels[1] = {CNC_SERVO_SIGNAL_ACT_COMPOSITE_VEL,
                              CNC_SERVO_UNIT_MM_PER_MIN, 1};
        return result;
    }

    CncServoSampleConfig positionConfig() const
    {
        CncServoSampleConfig result = config();
        result.loopType = CNC_SERVO_LOOP_POSITION;
        result.channelMask = (1U << 0) | (1U << 1) | (1U << 2);
        result.channels[0] = {CNC_SERVO_SIGNAL_CMD_POS, CNC_SERVO_UNIT_MM, 1};
        result.channels[1] = {CNC_SERVO_SIGNAL_ACT_POS, CNC_SERVO_UNIT_MM, 1};
        result.channels[2] = {CNC_SERVO_SIGNAL_FOLLOW_ERR, CNC_SERVO_UNIT_UM, 1};
        return result;
    }

    CncServoSampleStatus status() const
    {
        CncServoSampleStatus result{};
        EXPECT_TRUE(cncServoSampleReadStatus(&result));
        return result;
    }

    void registerConfig(const CncServoSampleConfig& sampleConfig)
    {
        uint32_t command = 0;
        ASSERT_TRUE(cncServoSampleRequestRegister(&sampleConfig, &command));
        ServoSampleSampler::instance().execute();
        const CncServoSampleStatus current = status();
        EXPECT_EQ(current.acknowledgedSequence, command);
    }

    void setProgramRunning(bool running)
    {
        setVarChanInt(0, VAR_CHAN_IS_RUNNING, running ? 1 : 0, 0);
        rt_data->chan_state[0][CHAN_IS_PARSERING] = running;
    }

    void clearMachineState()
    {
        std::memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
        std::memset(rt_data->axis_data, 0, sizeof(rt_data->axis_data));
        rt_data->chan_state[0][CHAN_IS_PARSERING] = false;
        setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
        setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 0.0, 0);
        for (int axis = 0; axis < 3; ++axis) {
            setVarAxisDouble(axis, VAR_AXIS_CMD_VEL, 0.0, 0);
        }

        ParamUnit cycle = {};
        cycle.value.val_uint = 1000;
        setParData(PARAMAN_FILE_NCU, 0, PAR_NCU_CYCLE, cycle);
    }
};

TEST_F(TestServoSample, RegisterAcceptsCurrentMemProgram)
{
    CncServoSampleConfig sampleConfig = config();
    sampleConfig.distanceMm = 0.0;
    sampleConfig.speedMmPerMin = 0.0;
    setProgramRunning(true);

    registerConfig(sampleConfig);

    const CncServoSampleStatus current = status();
    EXPECT_EQ(current.state, CNC_SERVO_STATE_WAITING_RESET);
    EXPECT_EQ(current.errorCode, CNC_SERVO_ERROR_NONE);
}

TEST_F(TestServoSample, DirectStartCapturesRunningProgramAndFinishesAtProgramEnd)
{
    registerConfig(config());
    setProgramRunning(true);

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().state, CNC_SERVO_STATE_RUNNING);
    const uint32_t countWhileRunning = status().sampleCount;
    EXPECT_TRUE(countWhileRunning > 0U);

    setProgramRunning(false);
    ServoSampleSampler::instance().execute();

    const CncServoSampleStatus completed = status();
    EXPECT_EQ(completed.acknowledgedSequence, command);
    EXPECT_EQ(completed.state, CNC_SERVO_STATE_COMPLETED);
    EXPECT_EQ(completed.stopReason, CNC_SERVO_STOP_PROGRAM_END);
    EXPECT_TRUE(completed.sampleCount > countWhileRunning);
}

TEST_F(TestServoSample, ResetThenCycleStartUsesGeneratedProgramPath)
{
    registerConfig(config());

    ServoSampleSampler::instance().notifyReset();
    ServoSampleSampler::instance().execute();
    const CncServoSampleStatus waitingProgram = status();
    ASSERT_EQ(waitingProgram.state, CNC_SERVO_STATE_WAITING_PROGRAM);
    ASSERT_NE(waitingProgram.resetSequence, 0U);

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestProgramReady(waitingProgram.resetSequence, &command));
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().acknowledgedSequence, command);
    EXPECT_EQ(status().state, CNC_SERVO_STATE_ARMED);

    setProgramRunning(true);
    ServoSampleSampler::instance().notifyCycleStart();
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().state, CNC_SERVO_STATE_RUNNING);

    setProgramRunning(false);
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().state, CNC_SERVO_STATE_COMPLETED);
    EXPECT_EQ(status().stopReason, CNC_SERVO_STOP_PROGRAM_END);
}

TEST_F(TestServoSample, SafetyFailureCanBeRetriedWithoutReregistering)
{
    registerConfig(config());
    rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = true;

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().acknowledgedSequence, command);
    EXPECT_EQ(status().state, CNC_SERVO_STATE_ERROR);
    EXPECT_EQ(status().stopReason, CNC_SERVO_STOP_SAFETY);
    EXPECT_EQ(status().errorCode, CNC_SERVO_ERROR_NOT_READY);

    rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = false;
    setProgramRunning(true);
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().state, CNC_SERVO_STATE_RUNNING);
    EXPECT_EQ(status().errorCode, CNC_SERVO_ERROR_NONE);
}

TEST_F(TestServoSample, ManualStopKeepsFinalSample)
{
    registerConfig(config());
    setProgramRunning(true);

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();
    const uint32_t beforeStop = status().sampleCount;
    ASSERT_TRUE(beforeStop > 0U);

    ASSERT_TRUE(cncServoSampleRequestStop(&command));
    ServoSampleSampler::instance().execute();

    const CncServoSampleStatus completed = status();
    EXPECT_EQ(completed.acknowledgedSequence, command);
    EXPECT_EQ(completed.state, CNC_SERVO_STATE_COMPLETED);
    EXPECT_EQ(completed.stopReason, CNC_SERVO_STOP_MANUAL);
    EXPECT_TRUE(completed.sampleCount > beforeStop);
}

TEST_F(TestServoSample, CompositeVelocityUsesThreeAxisActualMotion)
{
    registerConfig(compositeConfig());
    setProgramRunning(true);
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, 1200.0, 0);

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();

    rt_data->axis_data[0].motor_act_pos = 0.001;
    rt_data->axis_data[1].motor_act_pos = 0.002;
    rt_data->axis_data[2].motor_act_pos = 0.002;
    ServoSampleSampler::instance().execute();

    const CncServoSampleStatus current = status();
    ASSERT_TRUE(current.sampleCount >= 2U);
    const CncServoSampleFrame& sample =
        getCncServoSampleDataPtr()->samples[current.sampleCount - 1U];
    EXPECT_TRUE(std::fabs(sample.values[0] - 1200.0) < 1.0e-9);
    EXPECT_TRUE(std::fabs(sample.values[1] - 180.0) < 1.0e-9);
}

TEST_F(TestServoSample, PositionSignalsUseAxisPositionsInsteadOfMotorPositions)
{
    rt_data->axis_data[0].mac_cmd_pos = 12.5;
    rt_data->axis_data[0].fb_pos_no_comp = 12.375;
    rt_data->axis_data[0].motor_cmd_pos = 99.0;
    rt_data->axis_data[0].motor_act_pos = 98.0;
    registerConfig(positionConfig());
    setProgramRunning(true);

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();

    const CncServoSampleStatus current = status();
    ASSERT_TRUE(current.sampleCount > 0U);
    const CncServoSampleFrame& sample =
        getCncServoSampleDataPtr()->samples[current.sampleCount - 1U];
    EXPECT_DOUBLE_EQ(sample.values[0], 12.5);
    EXPECT_DOUBLE_EQ(sample.values[1], 12.375);
    EXPECT_DOUBLE_EQ(sample.values[2], 125.0);
    EXPECT_DOUBLE_EQ(current.originPositionMm, 12.5);
}

TEST_F(TestServoSample, CompositeSignalsRejectPositionLoopAndMixedChannels)
{
    CncServoSampleConfig invalid = compositeConfig();
    invalid.loopType = CNC_SERVO_LOOP_POSITION;
    registerConfig(invalid);
    EXPECT_EQ(status().state, CNC_SERVO_STATE_ERROR);
    EXPECT_EQ(status().errorCode, CNC_SERVO_ERROR_INVALID_CONFIG);

    invalid = compositeConfig();
    invalid.channels[1] = {CNC_SERVO_SIGNAL_ACT_VEL, CNC_SERVO_UNIT_MM_PER_MIN, 1};
    registerConfig(invalid);
    EXPECT_EQ(status().state, CNC_SERVO_STATE_ERROR);
    EXPECT_EQ(status().errorCode, CNC_SERVO_ERROR_INVALID_CONFIG);
}

TEST_F(TestServoSample, CompositeSafetyChecksAllThreeAxesAndCanRetry)
{
    registerConfig(compositeConfig());
    rt_data->axis_data[1].status.is_disable = true;

    uint32_t command = 0;
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().state, CNC_SERVO_STATE_ERROR);
    EXPECT_EQ(status().errorCode, CNC_SERVO_ERROR_NOT_READY);

    rt_data->axis_data[1].status.is_disable = false;
    setProgramRunning(true);
    ASSERT_TRUE(cncServoSampleRequestStart(&command));
    ServoSampleSampler::instance().execute();
    EXPECT_EQ(status().state, CNC_SERVO_STATE_RUNNING);
}

} // namespace
