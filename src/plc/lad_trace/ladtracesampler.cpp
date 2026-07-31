#include "ladtracesampler.h"

#include "ladtraceapi.h"
#include "regapi.h"

#include <cstring>

namespace {
template<typename T>
void storeRelease(T* destination, T value)
{
    __atomic_store_n(destination, value, __ATOMIC_RELEASE);
}
}

LadTraceSampler::LadTraceSampler(RegData& regData, CncLadTraceData& traceData)
    : _regData(regData)
    , _traceData(traceData)
{
    _lastCommandSequence = __atomic_load_n(
            &_traceData.acknowledgedSequence, __ATOMIC_ACQUIRE);
}

LadTraceSampler& LadTraceSampler::instance()
{
    static LadTraceSampler sampler(*getRegDataPtr(), *getCncLadTraceDataPtr());
    return sampler;
}

void LadTraceSampler::execute()
{
    const uint32_t sequence = __atomic_load_n(
            &_traceData.commandSequence, __ATOMIC_ACQUIRE);
    if (sequence != _lastCommandSequence) {
        const uint32_t command = __atomic_load_n(&_traceData.command, __ATOMIC_RELAXED);
        handleCommand(sequence, command);
        _lastCommandSequence = sequence;
        storeRelease(&_traceData.acknowledgedSequence, sequence);
    }

    traceCycle();
}

void LadTraceSampler::handleCommand(uint32_t sequence, uint32_t command)
{
    switch (command) {
    case CNC_LAD_TRACE_COMMAND_START:
        start(sequence);
        break;
    case CNC_LAD_TRACE_COMMAND_STOP:
        stop();
        break;
    default:
        break;
    }
}

void LadTraceSampler::start(uint32_t sequence)
{
    std::memcpy(&_config, &_traceData.config, sizeof(_config));
    _ticksUntilSample = 0;
    _sampleCount = 0;
    storeRelease(&_traceData.sampleCount, 0U);
    storeRelease(&_traceData.sessionSequence, sequence);
    storeRelease(&_traceData.stopReason,
                 static_cast<uint32_t>(CNC_LAD_TRACE_STOP_REASON_NONE));
    storeRelease(&_traceData.errorCode,
                 static_cast<int32_t>(CNC_LAD_TRACE_ERROR_NONE));

    const CncLadTraceError error = validateConfig();
    if (error != CNC_LAD_TRACE_ERROR_NONE) {
        fail(error);
        return;
    }

    if (_config.startCondition == CNC_LAD_TRACE_START_TRIGGER) {
        _startState = readSignal(_config.startTrigger);
        storeRelease(&_traceData.state,
                     static_cast<uint32_t>(CNC_LAD_TRACE_STATE_WAITING_TRIGGER));
    } else {
        if (_config.stopCondition == CNC_LAD_TRACE_STOP_TRIGGER) {
            _stopState = readSignal(_config.stopTrigger);
        }
        storeRelease(&_traceData.state,
                     static_cast<uint32_t>(CNC_LAD_TRACE_STATE_CAPTURING));
    }
}

void LadTraceSampler::stop()
{
    const uint32_t state = __atomic_load_n(&_traceData.state, __ATOMIC_ACQUIRE);
    if (state == CNC_LAD_TRACE_STATE_WAITING_TRIGGER ||
            state == CNC_LAD_TRACE_STATE_CAPTURING) {
        finish(CNC_LAD_TRACE_STOP_REASON_MANUAL);
    }
}

void LadTraceSampler::traceCycle()
{
    uint32_t state = __atomic_load_n(&_traceData.state, __ATOMIC_ACQUIRE);
    if (state == CNC_LAD_TRACE_STATE_WAITING_TRIGGER) {
        const bool current = readSignal(_config.startTrigger);
        const bool matched = triggerMatches(_startState, current,
                                            _config.startTriggerMode);
        _startState = current;
        if (!matched) {
            return;
        }

        if (_config.stopCondition == CNC_LAD_TRACE_STOP_TRIGGER) {
            _stopState = readSignal(_config.stopTrigger);
        }
        storeRelease(&_traceData.state,
                     static_cast<uint32_t>(CNC_LAD_TRACE_STATE_CAPTURING));
        state = CNC_LAD_TRACE_STATE_CAPTURING;
    }

    if (state != CNC_LAD_TRACE_STATE_CAPTURING) {
        return;
    }

    bool stopTriggered = false;
    if (_config.stopCondition == CNC_LAD_TRACE_STOP_TRIGGER) {
        const bool current = readSignal(_config.stopTrigger);
        stopTriggered = triggerMatches(_stopState, current, _config.stopTriggerMode);
        _stopState = current;
    }

    const bool sampleDue = _ticksUntilSample == 0U;
    if (!sampleDue && !stopTriggered) {
        --_ticksUntilSample;
        return;
    }
    if (sampleDue) {
        _ticksUntilSample = _config.periodMs - 1U;
    }

    const bool full = appendSample(sampleChannels());
    if (stopTriggered) {
        finish(CNC_LAD_TRACE_STOP_REASON_TRIGGER);
    } else if (full) {
        finish(CNC_LAD_TRACE_STOP_REASON_BUFFER_FULL);
    }
}

void LadTraceSampler::finish(CncLadTraceStopReason reason)
{
    storeRelease(&_traceData.stopReason, static_cast<uint32_t>(reason));
    storeRelease(&_traceData.state,
                 static_cast<uint32_t>(CNC_LAD_TRACE_STATE_COMPLETED));
}

void LadTraceSampler::fail(CncLadTraceError error)
{
    storeRelease(&_traceData.stopReason,
                 static_cast<uint32_t>(CNC_LAD_TRACE_STOP_REASON_INVALID_CONFIG));
    storeRelease(&_traceData.errorCode, static_cast<int32_t>(error));
    storeRelease(&_traceData.state,
                 static_cast<uint32_t>(CNC_LAD_TRACE_STATE_ERROR));
}

uint8_t LadTraceSampler::sampleChannels() const
{
    uint8_t result = 0;
    for (int channel = 0; channel < CNC_LAD_TRACE_CHANNEL_COUNT; ++channel) {
        if (_config.channels[channel].enabled && readSignal(_config.channels[channel])) {
            result |= static_cast<uint8_t>(1U << channel);
        }
    }
    return result;
}

bool LadTraceSampler::appendSample(uint8_t sample)
{
    if (_sampleCount >= CNC_LAD_TRACE_SAMPLE_CAPACITY) {
        return true;
    }
    _traceData.samples[_sampleCount] = sample;
    ++_sampleCount;
    storeRelease(&_traceData.sampleCount, _sampleCount);
    return _sampleCount >= CNC_LAD_TRACE_SAMPLE_CAPACITY;
}

bool LadTraceSampler::readSignal(const CncLadTraceSignal& signal) const
{
    uint32_t value = 0;
    switch (static_cast<RegType>(signal.type)) {
    case REG_A: value = static_cast<uint8_t>(_regData.regA[signal.index]); break;
    case REG_R: value = static_cast<uint8_t>(_regData.regR[signal.index]); break;
    case REG_X: value = static_cast<uint8_t>(_regData.regX[signal.index]); break;
    case REG_Y: value = static_cast<uint8_t>(_regData.regY[signal.index]); break;
    case REG_F: value = static_cast<uint8_t>(_regData.regF[signal.index]); break;
    case REG_G: value = static_cast<uint8_t>(_regData.regG[signal.index]); break;
    case REG_I: value = static_cast<uint8_t>(_regData.regI[signal.index]); break;
    case REG_Q: value = static_cast<uint8_t>(_regData.regQ[signal.index]); break;
    case REG_K: value = static_cast<uint8_t>(_regData.regK[signal.index]); break;
    case REG_B: value = static_cast<uint32_t>(_regData.regB[signal.index]); break;
    case REG_D: value = static_cast<uint32_t>(_regData.regD[signal.index]); break;
    case REG_T: value = static_cast<uint32_t>(_regData.regT[signal.index]); break;
    case REG_C: value = static_cast<uint32_t>(_regData.regC[signal.index]); break;
    default: return false;
    }
    return ((value >> signal.bit) & 1U) != 0U;
}

CncLadTraceError LadTraceSampler::validateConfig() const
{
    if (_config.periodMs < 1U || _config.periodMs > 1000U) {
        return CNC_LAD_TRACE_ERROR_PERIOD;
    }

    bool hasChannel = false;
    for (int channel = 0; channel < CNC_LAD_TRACE_CHANNEL_COUNT; ++channel) {
        if (!_config.channels[channel].enabled) {
            continue;
        }
        hasChannel = true;
        if (!signalValid(_config.channels[channel])) {
            return CNC_LAD_TRACE_ERROR_SIGNAL;
        }
    }
    if (!hasChannel) {
        return CNC_LAD_TRACE_ERROR_NO_CHANNEL;
    }

    if (_config.startCondition > CNC_LAD_TRACE_START_TRIGGER) {
        return CNC_LAD_TRACE_ERROR_START_TRIGGER;
    }
    if (_config.startCondition == CNC_LAD_TRACE_START_TRIGGER &&
            (!signalValid(_config.startTrigger) ||
             _config.startTriggerMode > CNC_LAD_TRACE_TRIGGER_OFF)) {
        return CNC_LAD_TRACE_ERROR_START_TRIGGER;
    }

    if (_config.stopCondition > CNC_LAD_TRACE_STOP_TRIGGER) {
        return CNC_LAD_TRACE_ERROR_STOP_TRIGGER;
    }
    if (_config.stopCondition == CNC_LAD_TRACE_STOP_TRIGGER &&
            (!signalValid(_config.stopTrigger) ||
             _config.stopTriggerMode > CNC_LAD_TRACE_TRIGGER_CHANGE)) {
        return CNC_LAD_TRACE_ERROR_STOP_TRIGGER;
    }
    return CNC_LAD_TRACE_ERROR_NONE;
}

bool LadTraceSampler::signalValid(const CncLadTraceSignal& signal)
{
    int count = 0;
    int bitCount = 0;
    switch (static_cast<RegType>(signal.type)) {
    case REG_A: count = REG_A_NUM; bitCount = 8; break;
    case REG_R: count = REG_R_NUM; bitCount = 8; break;
    case REG_X: count = REG_X_NUM; bitCount = 8; break;
    case REG_Y: count = REG_Y_NUM; bitCount = 8; break;
    case REG_F: count = REG_F_NUM; bitCount = 8; break;
    case REG_G: count = REG_G_NUM; bitCount = 8; break;
    case REG_I: count = REG_I_NUM; bitCount = 8; break;
    case REG_Q: count = REG_Q_NUM; bitCount = 8; break;
    case REG_K: count = REG_K_NUM; bitCount = 8; break;
    case REG_B: count = REG_B_NUM; bitCount = 32; break;
    case REG_D: count = REG_D_NUM; bitCount = 32; break;
    case REG_T: count = REG_T_NUM; bitCount = 32; break;
    case REG_C: count = REG_C_NUM; bitCount = 32; break;
    default: return false;
    }
    return signal.index >= 0 && signal.index < count &&
            signal.bit >= 0 && signal.bit < bitCount;
}

bool LadTraceSampler::triggerMatches(bool previous, bool current, uint32_t mode)
{
    switch (mode) {
    case CNC_LAD_TRACE_TRIGGER_RISING: return !previous && current;
    case CNC_LAD_TRACE_TRIGGER_FALLING: return previous && !current;
    case CNC_LAD_TRACE_TRIGGER_CHANGE: return previous != current;
    case CNC_LAD_TRACE_TRIGGER_ON: return current;
    case CNC_LAD_TRACE_TRIGGER_OFF: return !current;
    default: return false;
    }
}
