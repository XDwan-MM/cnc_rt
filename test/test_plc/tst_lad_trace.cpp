#include <gtest/gtest.h>

#include <cstring>

#include "ladtraceapi.h"
#include "ladtracesampler.h"
#include "regapi.h"
#include "test_env.h"

namespace {

class TestLadTrace : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!testEnvReady()) {
            GTEST_SKIP() << testEnvMessage();
        }

        std::memset(getCncLadTraceDataPtr(), 0, sizeof(CncLadTraceData));
        RegData* registers = getRegDataPtr();
        ASSERT_TRUE(registers != nullptr);
        registers->regR[0] = 0;
    }
};

TEST_F(TestLadTrace, PeriodicCaptureAndManualStop)
{
    CncLadTraceData* data = getCncLadTraceDataPtr();
    RegData* registers = getRegDataPtr();
    ASSERT_TRUE(data != nullptr);
    ASSERT_TRUE(registers != nullptr);

    CncLadTraceConfig config{};
    config.periodMs = 1;
    config.startCondition = CNC_LAD_TRACE_START_PERIODIC;
    config.stopCondition = CNC_LAD_TRACE_STOP_BUFFER_FULL;
    config.channels[0] = {REG_R, 0, 0, 1};
    registers->regR[0] = 1;

    LadTraceSampler sampler(*registers, *data);
    uint32_t command = 0;
    ASSERT_TRUE(cncLadTraceRequestStart(&config, &command));
    sampler.execute();

    CncLadTraceStatus status{};
    ASSERT_TRUE(cncLadTraceReadStatus(&status));
    EXPECT_EQ(status.acknowledgedSequence, command);
    EXPECT_EQ(status.state, CNC_LAD_TRACE_STATE_CAPTURING);
    EXPECT_EQ(status.sampleCount, 1U);
    EXPECT_EQ(data->samples[0], 1U);

    ASSERT_TRUE(cncLadTraceRequestStop(&command));
    sampler.execute();
    ASSERT_TRUE(cncLadTraceReadStatus(&status));
    EXPECT_EQ(status.acknowledgedSequence, command);
    EXPECT_EQ(status.state, CNC_LAD_TRACE_STATE_COMPLETED);
    EXPECT_EQ(status.stopReason, CNC_LAD_TRACE_STOP_REASON_MANUAL);
    EXPECT_EQ(status.sampleCount, 1U);
}

} // namespace
