#include "servosamplesampler.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "global.h"
#include "servosampleapi.h"

namespace {
constexpr double kMinVelocity = 1.0e-9;
constexpr uint32_t kCompositeAxisCount = 3U;

uint32_t loadState(const CncServoSampleData* data)
{
    return __atomic_load_n(&data->state, __ATOMIC_ACQUIRE);
}

uint32_t loadSampleCount(const CncServoSampleData* data)
{
    return __atomic_load_n(&data->sampleCount, __ATOMIC_RELAXED);
}

uint32_t nextNonZero(uint32_t value)
{
    ++value;
    return value == 0U ? 1U : value;
}
}

ServoSampleSampler& ServoSampleSampler::instance()
{
    static ServoSampleSampler sampler;
    return sampler;
}

void ServoSampleSampler::notifyReset()
{
    _resetPending = true;
}

void ServoSampleSampler::notifyCycleStart()
{
    _cycleStartPending = true;
}

void ServoSampleSampler::ensureShared()
{
    if (!_shared) {
        _shared = getCncServoSampleDataPtr();
    }
    if (!_shared) return;

    const uint32_t version = __atomic_load_n(&_shared->protocolVersion, __ATOMIC_ACQUIRE);
    if (version == CNC_SERVO_SAMPLE_PROTOCOL_VERSION) return;

    // The HMI and realtime process must use the same ABI. Reset once on startup
    // instead of interpreting a stale v1 layout as a live capture session.
    std::memset(_shared, 0, sizeof(*_shared));
    __atomic_store_n(&_shared->protocolVersion, CNC_SERVO_SAMPLE_PROTOCOL_VERSION,
                     __ATOMIC_RELEASE);
}

void ServoSampleSampler::execute()
{
    ensureShared();
    if (!_shared) return;

    handleCommand();
    handleReset();
    handleCycleStart();

    const uint32_t state = loadState(_shared);
    if ((state == CNC_SERVO_STATE_RUNNING || state == CNC_SERVO_STATE_DRAINING) &&
        !machineReady(_config)) {
        finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_SAFETY,
               CNC_SERVO_ERROR_NOT_READY);
        return;
    }

    if (state == CNC_SERVO_STATE_RUNNING) {
        if (isCompositeConfig(_config)) {
            _actualCompositeVelocityMmPerMin = actualCompositeVelocityMmPerMin();
        } else {
            const double actualPosition = rt_data->axis_data[_config.axis].motor_act_pos;
            _actualVelocityMmPerMin = actualVelocityMmPerMin(actualPosition);
        }
        sampleIfDue();

        const bool motionObserved = isCompositeConfig(_config)
            ? std::fabs(var_chan[0][VAR_CHAN_CMD_FEEDRATE].value.f) > kMinVelocity ||
              std::fabs(_actualCompositeVelocityMmPerMin) > kMinVelocity
            : std::fabs(var_axis[_config.axis][VAR_AXIS_CMD_VEL].value.f) > kMinVelocity ||
              std::fabs(rt_data->axis_data[_config.axis].mac_cmd_pos -
                        rt_data->axis_data[_config.axis].fb_pos_no_comp) > kMinVelocity;
        if (programActive() || motionObserved) {
            _programObserved = true;
        }
        if (_programObserved && programFinished()) {
            sampleOne();
            if (loadState(_shared) == CNC_SERVO_STATE_RUNNING) {
                finish(CNC_SERVO_STATE_COMPLETED, CNC_SERVO_STOP_PROGRAM_END,
                       CNC_SERVO_ERROR_NONE);
            }
        }
    } else if (state == CNC_SERVO_STATE_DRAINING && _drainRequested) {
        sampleOne();
        _drainRequested = 0;
        if (loadState(_shared) == CNC_SERVO_STATE_DRAINING) {
            finish(CNC_SERVO_STATE_COMPLETED, CNC_SERVO_STOP_MANUAL,
                   CNC_SERVO_ERROR_NONE);
        }
    }
}

void ServoSampleSampler::handleCommand()
{
    const uint32_t sequence = __atomic_load_n(&_shared->commandSequence, __ATOMIC_ACQUIRE);
    if (sequence == _lastCommandSequence) return;

    _lastCommandSequence = sequence;
    const uint32_t command = __atomic_load_n(&_shared->command, __ATOMIC_RELAXED);
    switch (command) {
    case CNC_SERVO_COMMAND_REGISTER:
        registerConfig(_shared->config, sequence);
        break;
    case CNC_SERVO_COMMAND_PROGRAM_READY:
        acknowledgeProgram(sequence);
        break;
    case CNC_SERVO_COMMAND_START: {
        const uint32_t state = loadState(_shared);
        if (!_configured || state == CNC_SERVO_STATE_IDLE) {
            finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_UNREGISTERED,
                   CNC_SERVO_ERROR_PROGRAM_NOT_READY);
        } else if (state == CNC_SERVO_STATE_RUNNING || state == CNC_SERVO_STATE_DRAINING) {
            __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_BUSY, __ATOMIC_RELEASE);
        } else {
            startCapture();
        }
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        break;
    }
    case CNC_SERVO_COMMAND_STOP: {
        const uint32_t state = loadState(_shared);
        if (state == CNC_SERVO_STATE_RUNNING) {
            _drainRequested = 1;
            __atomic_store_n(&_shared->state, CNC_SERVO_STATE_DRAINING, __ATOMIC_RELEASE);
        } else if (state == CNC_SERVO_STATE_DRAINING) {
            // The pending final sample will complete the stop request.
        } else if (state != CNC_SERVO_STATE_IDLE) {
            finish(CNC_SERVO_STATE_COMPLETED, CNC_SERVO_STOP_MANUAL,
                   CNC_SERVO_ERROR_NONE);
        }
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        break;
    }
    case CNC_SERVO_COMMAND_CLEAR: {
        const uint32_t state = loadState(_shared);
        if (state == CNC_SERVO_STATE_RUNNING || state == CNC_SERVO_STATE_DRAINING) {
            __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_BUSY, __ATOMIC_RELEASE);
        } else {
            resetCaptureData();
            __atomic_store_n(&_shared->stopReason, CNC_SERVO_STOP_NONE, __ATOMIC_RELEASE);
            __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
            __atomic_store_n(&_shared->state,
                             _configured ? CNC_SERVO_STATE_WAITING_RESET : CNC_SERVO_STATE_IDLE,
                             __ATOMIC_RELEASE);
        }
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        break;
    }
    case CNC_SERVO_COMMAND_UNREGISTER:
        if (loadState(_shared) == CNC_SERVO_STATE_RUNNING ||
            loadState(_shared) == CNC_SERVO_STATE_DRAINING) {
            __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_BUSY, __ATOMIC_RELEASE);
        } else {
            _configured = false;
            _resetPending = false;
            _cycleStartPending = false;
            _programObserved = false;
            __atomic_store_n(&_shared->state, CNC_SERVO_STATE_IDLE, __ATOMIC_RELEASE);
        }
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        break;
    default:
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        break;
    }
}

void ServoSampleSampler::handleReset()
{
    if (!_resetPending) return;
    _resetPending = false;
    if (!_configured || loadState(_shared) == CNC_SERVO_STATE_IDLE) return;

    resetCaptureData();
    const uint32_t resetSequence = nextNonZero(
        __atomic_load_n(&_shared->resetSequence, __ATOMIC_RELAXED));
    __atomic_store_n(&_shared->resetSequence, resetSequence, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->programSequence, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->stopReason, CNC_SERVO_STOP_RESET, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->state, CNC_SERVO_STATE_WAITING_PROGRAM,
                     __ATOMIC_RELEASE);
}

void ServoSampleSampler::handleCycleStart()
{
    if (!_cycleStartPending) return;
    _cycleStartPending = false;
    if (loadState(_shared) == CNC_SERVO_STATE_ARMED) {
        startCapture();
    }
}

bool ServoSampleSampler::registerConfig(const CncServoSampleConfig& config,
                                        uint32_t sequence)
{
    int32_t error = CNC_SERVO_ERROR_NONE;
    const uint32_t state = loadState(_shared);
    if (state == CNC_SERVO_STATE_RUNNING || state == CNC_SERVO_STATE_DRAINING) {
        __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_BUSY, __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        return false;
    }
    if (!validateConfig(config, &error)) {
        _configured = false;
        _programObserved = false;
        __atomic_store_n(&_shared->stopReason, CNC_SERVO_STOP_INVALID_CONFIG,
                         __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->errorCode, error, __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->state, CNC_SERVO_STATE_ERROR, __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        return false;
    }

    _config = config;
    const uint32_t cycleUs = par_ncu[PAR_NCU_CYCLE].value.val_uint > 0
            ? par_ncu[PAR_NCU_CYCLE].value.val_uint : 1000U;
    const uint64_t requestedUs = static_cast<uint64_t>(config.requestedPeriodMs) * 1000ULL;
    _effectiveTicks = static_cast<uint32_t>((requestedUs + cycleUs - 1U) / cycleUs);
    _effectiveTicks = std::max<uint32_t>(_effectiveTicks, 1U);
    _config.effectivePeriodTicks = _effectiveTicks;
    _shared->config = _config;
    _configured = true;
    _programObserved = false;
    _drainRequested = 0;
    resetCaptureData();
    __atomic_store_n(&_shared->effectivePeriodTicks, _effectiveTicks, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->effectivePeriodNs,
                     static_cast<uint64_t>(_effectiveTicks) * cycleUs * 1000ULL,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->stopReason, CNC_SERVO_STOP_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->state, CNC_SERVO_STATE_WAITING_RESET,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
    return true;
}

void ServoSampleSampler::acknowledgeProgram(uint32_t sequence)
{
    const uint32_t requested = __atomic_load_n(&_shared->requestedProgramSequence,
                                               __ATOMIC_ACQUIRE);
    const uint32_t resetSequence = __atomic_load_n(&_shared->resetSequence,
                                                   __ATOMIC_ACQUIRE);
    const uint32_t state = loadState(_shared);
    if (!_configured || !rt_data || (state != CNC_SERVO_STATE_WAITING_PROGRAM &&
                                     state != CNC_SERVO_STATE_ARMED) || requested == 0U ||
        requested != resetSequence) {
        __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_PROGRAM_NOT_READY,
                         __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        return;
    }

    rt_data->target_mode = MODE_MDI;
    __atomic_store_n(&_shared->programSequence, requested, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->state, CNC_SERVO_STATE_ARMED, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
}

bool ServoSampleSampler::startCapture()
{
    if (!_configured) return false;
    if (!machineReady(_config)) {
        finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_SAFETY,
               CNC_SERVO_ERROR_NOT_READY);
        return false;
    }
    resetCaptureData();
    _sessionSequence = nextNonZero(_sessionSequence);
    __atomic_store_n(&_shared->sessionSequence, _sessionSequence, __ATOMIC_RELEASE);
    _shared->originPositionMm = isCompositeConfig(_config)
        ? 0.0 : rt_data->axis_data[_config.axis].mac_cmd_pos;
    _ticksUntilSample = 0;
    _drainRequested = 0;
    _programObserved = false;
    _hasLastActualPosition = false;
    _actualVelocityMmPerMin = 0.0;
    _hasLastCompositePosition = false;
    _actualCompositeVelocityMmPerMin = 0.0;
    __atomic_store_n(&_shared->stopReason, CNC_SERVO_STOP_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->state, CNC_SERVO_STATE_RUNNING, __ATOMIC_RELEASE);
    return true;
}

void ServoSampleSampler::resetCaptureData()
{
    _ticksUntilSample = 0;
    _drainRequested = 0;
    _programObserved = false;
    _hasLastActualPosition = false;
    _actualVelocityMmPerMin = 0.0;
    _hasLastCompositePosition = false;
    _actualCompositeVelocityMmPerMin = 0.0;
    _shared->originPositionMm = 0.0;
    __atomic_store_n(&_shared->sampleCount, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->droppedCount, 0U, __ATOMIC_RELEASE);
}

bool ServoSampleSampler::validateConfig(const CncServoSampleConfig& config,
                                        int32_t* error) const
{
    if (config.axis >= SYS_AXES_NUM) {
        if (error) *error = CNC_SERVO_ERROR_INVALID_AXIS;
        return false;
    }
    if (config.loopType != CNC_SERVO_LOOP_POSITION &&
        config.loopType != CNC_SERVO_LOOP_VELOCITY) {
        if (error) *error = CNC_SERVO_ERROR_INVALID_LOOP;
        return false;
    }
    if (config.requestedPeriodMs < 1 || config.requestedPeriodMs > 1000) {
        if (error) *error = CNC_SERVO_ERROR_INVALID_PERIOD;
        return false;
    }
    if (config.capacity < 1 || config.capacity > CNC_SERVO_SAMPLE_CAPACITY) {
        if (error) *error = CNC_SERVO_ERROR_INVALID_CAPACITY;
        return false;
    }
    uint32_t validatedMask = 0;
    bool hasAxisSignal = false;
    bool hasCompositeSignal = false;
    for (uint32_t i = 0; i < CNC_SERVO_SAMPLE_CHANNEL_COUNT; ++i) {
        const CncServoSampleChannel& channel = config.channels[i];
        if (!channel.enabled) continue;
        if (channel.signalId < CNC_SERVO_SIGNAL_CMD_POS ||
            channel.signalId > CNC_SERVO_SIGNAL_ACT_COMPOSITE_VEL) {
            if (error) *error = CNC_SERVO_ERROR_INVALID_CONFIG;
            return false;
        }
        if (channel.signalId >= CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL) {
            hasCompositeSignal = true;
        } else {
            hasAxisSignal = true;
        }
        validatedMask |= (1U << i);
    }
    if (validatedMask == 0 || validatedMask != config.channelMask) {
        if (error) *error = CNC_SERVO_ERROR_NO_CHANNEL;
        return false;
    }
    if ((hasCompositeSignal && hasAxisSignal) ||
        (hasCompositeSignal && config.loopType != CNC_SERVO_LOOP_VELOCITY)) {
        if (error) *error = CNC_SERVO_ERROR_INVALID_CONFIG;
        return false;
    }
    return true;
}

bool ServoSampleSampler::machineReady(const CncServoSampleConfig& config) const
{
    if (!rt_data || rt_get_sys_state(SYS_IS_ESTOP_ACTIVE) ||
        rt_get_sys_state(SYS_IS_ALARMING)) {
        return false;
    }
    if (!isCompositeConfig(config)) {
        return config.axis < SYS_AXES_NUM &&
               !rt_data->axis_data[config.axis].status.is_disable;
    }
    for (uint32_t axis = 0; axis < kCompositeAxisCount; ++axis) {
        if (axis >= SYS_AXES_NUM || rt_data->axis_data[axis].status.is_disable) {
            return false;
        }
    }
    return true;
}

bool ServoSampleSampler::isCompositeConfig(const CncServoSampleConfig& config) const
{
    for (uint32_t i = 0; i < CNC_SERVO_SAMPLE_CHANNEL_COUNT; ++i) {
        if (config.channels[i].enabled &&
            config.channels[i].signalId >= CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL) {
            return true;
        }
    }
    return false;
}

bool ServoSampleSampler::programActive() const
{
    return var_chan[0][VAR_CHAN_IS_RUNNING].value.i != 0 ||
           rt_get_chan_state(0, CHAN_IS_PARSERING);
}

bool ServoSampleSampler::programFinished() const
{
    return !programActive();
}

double ServoSampleSampler::actualVelocityMmPerMin(double actualPosition)
{
    const uint32_t cycleUs = par_ncu[PAR_NCU_CYCLE].value.val_uint > 0
            ? par_ncu[PAR_NCU_CYCLE].value.val_uint : 1000U;
    double velocity = 0.0;
    if (_hasLastActualPosition) {
        velocity = (actualPosition - _lastActualPosition) /
                   (static_cast<double>(cycleUs) / TIME_US_2_S) * 60.0;
    }
    _lastActualPosition = actualPosition;
    _hasLastActualPosition = true;
    return velocity;
}

double ServoSampleSampler::actualCompositeVelocityMmPerMin()
{
    const uint32_t cycleUs = par_ncu[PAR_NCU_CYCLE].value.val_uint > 0
            ? par_ncu[PAR_NCU_CYCLE].value.val_uint : 1000U;
    double velocitySquared = 0.0;
    for (uint32_t axis = 0; axis < kCompositeAxisCount; ++axis) {
        const double position = rt_data->axis_data[axis].motor_act_pos;
        if (_hasLastCompositePosition) {
            const double velocity = (position - _lastCompositePosition[axis]) /
                    (static_cast<double>(cycleUs) / TIME_US_2_S) * 60.0;
            velocitySquared += velocity * velocity;
        }
        _lastCompositePosition[axis] = position;
    }
    _hasLastCompositePosition = true;
    return std::sqrt(velocitySquared);
}

void ServoSampleSampler::sampleIfDue()
{
    if (_ticksUntilSample > 0) {
        --_ticksUntilSample;
        return;
    }
    sampleOne();
    _ticksUntilSample = _effectiveTicks > 0 ? _effectiveTicks - 1U : 0U;
}

void ServoSampleSampler::sampleOne()
{
    const uint32_t count = loadSampleCount(_shared);
    if (count >= _config.capacity || count >= CNC_SERVO_SAMPLE_CAPACITY) {
        finish(CNC_SERVO_STATE_COMPLETED, CNC_SERVO_STOP_BUFFER_FULL,
               CNC_SERVO_ERROR_NONE);
        return;
    }

    CncServoSampleFrame frame{};
    const double cmdPos = rt_data->axis_data[_config.axis].mac_cmd_pos;
    const double actPos = rt_data->axis_data[_config.axis].fb_pos_no_comp;
    const double cmdVel = var_axis[_config.axis][VAR_AXIS_CMD_VEL].value.f * 60.0;
    const double cmdCompositeVel = var_chan[0][VAR_CHAN_CMD_FEEDRATE].value.f;
    const double followErrUm = (cmdPos - actPos) * 1000.0;
    for (uint32_t i = 0; i < CNC_SERVO_SAMPLE_CHANNEL_COUNT; ++i) {
        if (!_config.channels[i].enabled) continue;
        switch (_config.channels[i].signalId) {
        case CNC_SERVO_SIGNAL_CMD_POS: frame.values[i] = cmdPos; break;
        case CNC_SERVO_SIGNAL_ACT_POS: frame.values[i] = actPos; break;
        case CNC_SERVO_SIGNAL_FOLLOW_ERR: frame.values[i] = followErrUm; break;
        case CNC_SERVO_SIGNAL_CMD_VEL: frame.values[i] = cmdVel; break;
        case CNC_SERVO_SIGNAL_ACT_VEL: frame.values[i] = _actualVelocityMmPerMin; break;
        case CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL: frame.values[i] = cmdCompositeVel; break;
        case CNC_SERVO_SIGNAL_ACT_COMPOSITE_VEL:
            frame.values[i] = _actualCompositeVelocityMmPerMin;
            break;
        default: break;
        }
    }
    _shared->samples[count] = frame;
    __atomic_store_n(&_shared->sampleCount, count + 1U, __ATOMIC_RELEASE);
}

void ServoSampleSampler::finish(uint32_t state, uint32_t reason, int32_t error)
{
    __atomic_store_n(&_shared->stopReason, reason, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, error, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->state, state, __ATOMIC_RELEASE);
    _programObserved = false;
    _drainRequested = 0;
}

extern "C" void cncServoSampleNotifyReset(void)
{
    ServoSampleSampler::instance().notifyReset();
}

extern "C" void cncServoSampleNotifyCycleStart(void)
{
    ServoSampleSampler::instance().notifyCycleStart();
}
