#pragma once

#include "ladtracedata.h"
#include "regdata.h"

#include <cstdint>

class LadTraceSampler {
public:
    LadTraceSampler(RegData& regData, CncLadTraceData& traceData);

    static LadTraceSampler& instance();
    void execute();

private:
    RegData& _regData;
    CncLadTraceData& _traceData;
    CncLadTraceConfig _config{};
    uint32_t _lastCommandSequence = 0;
    uint32_t _ticksUntilSample = 0;
    uint32_t _sampleCount = 0;
    bool _startState = false;
    bool _stopState = false;

    void handleCommand(uint32_t sequence, uint32_t command);
    void start(uint32_t sequence);
    void stop();
    void traceCycle();
    void finish(CncLadTraceStopReason reason);
    void fail(CncLadTraceError error);
    uint8_t sampleChannels() const;
    bool appendSample(uint8_t sample);
    bool readSignal(const CncLadTraceSignal& signal) const;
    CncLadTraceError validateConfig() const;

    static bool signalValid(const CncLadTraceSignal& signal);
    static bool triggerMatches(bool previous, bool current, uint32_t mode);
};
