#ifndef CNC_SERVO_SAMPLE_SAMPLER_H
#define CNC_SERVO_SAMPLE_SAMPLER_H

#include <stdint.h>

#include "servosampledata.h"

class ServoSampleSampler
{
public:
    static ServoSampleSampler& instance();
    void execute();
    void notifyReset();
    void notifyCycleStart();

private:
    ServoSampleSampler() = default;
    ServoSampleSampler(const ServoSampleSampler&) = delete;
    ServoSampleSampler& operator=(const ServoSampleSampler&) = delete;

    void ensureShared();
    void handleCommand();
    void handleReset();
    void handleCycleStart();
    bool registerConfig(const CncServoSampleConfig& config, uint32_t sequence);
    void acknowledgeProgram(uint32_t sequence);
    bool startCapture();
    void resetCaptureData();
    void sampleIfDue();
    void sampleOne();
    void finish(uint32_t state, uint32_t reason, int32_t error);
    bool validateConfig(const CncServoSampleConfig& config, int32_t* error) const;
    bool machineReady(const CncServoSampleConfig& config) const;
    bool isCompositeConfig(const CncServoSampleConfig& config) const;
    bool programActive() const;
    bool programFinished() const;
    double actualVelocityMmPerMin(double actualPosition);
    double actualCompositeVelocityMmPerMin();

    CncServoSampleData* _shared = nullptr;
    CncServoSampleConfig _config{};
    uint32_t _lastCommandSequence = 0;
    uint32_t _sessionSequence = 0;
    uint32_t _ticksUntilSample = 0;
    uint32_t _effectiveTicks = 1;
    uint32_t _drainRequested = 0;
    bool _configured = false;
    bool _resetPending = false;
    bool _cycleStartPending = false;
    bool _programObserved = false;
    bool _hasLastActualPosition = false;
    double _lastActualPosition = 0.0;
    double _actualVelocityMmPerMin = 0.0;
    bool _hasLastCompositePosition = false;
    double _lastCompositePosition[3] = {0.0, 0.0, 0.0};
    double _actualCompositeVelocityMmPerMin = 0.0;
};

#endif
