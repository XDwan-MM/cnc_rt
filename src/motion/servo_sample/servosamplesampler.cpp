#include "servosamplesampler.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "global.h"
#include "servosampleapi.h"
#include "tapreturn.h"

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

    // HMI 与实时进程必须使用相同的 ABI。启动时只复位一次，
    // 避免把残留的 v1 布局误认为正在进行的采样会话。
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
        const bool tapMotion = isRigidTapConfig(_config) && cycle_tp && cycle_tp->plan_block &&
            (cycle_tp->plan_block->type == RIGID_TAP || cycle_tp->plan_block->type == TAP_RETURN);
        _tapMotionActive = tapMotion;
        if (isRigidTapConfig(_config)) {
            // HMI 发出开始请求后立即采样。攻丝程序块只控制同步误差积分，
            // 基本的速度、转速和角度反馈在进入 G74/G84 前也会采集。
            __atomic_store_n(&_shared->samplePhase,
                             tapMotion
                                 ? (cycle_tp->plan_block->type == TAP_RETURN
                                        ? CNC_SERVO_SAMPLE_PHASE_RETURN
                                        : CNC_SERVO_SAMPLE_PHASE_MOTION)
                                 : CNC_SERVO_SAMPLE_PHASE_WAIT_MOTION,
                             __ATOMIC_RELEASE);
        }
        if (isExternalMotionConfig(_config) && !_circleCaptureStarted) {
            /* 生成的圆度程序包含直线预定位移动。不要把该移动纳入圆拟合，
             * 只有规划器进入圆弧后才打开采样窗口。 */
            if (!isCircleMotionActive()) return;
            _circleCaptureStarted = true;
            _circleArcBlockCount = 1;
            _circleArcLineNumber = cycle_tp->plan_block->line_number;
            _ticksUntilSample = 0;
            _hasLastActualPosition = false;
            _actualVelocityMmPerMin = 0.0;
            _hasLastCompositePosition = false;
            _actualCompositeVelocityMmPerMin = 0.0;
        } else if (isExternalMotionConfig(_config)) {
            if (!isCircleMotionActive()) {
                /* 生成的测试程序恰好包含两段半圆。离开第二段圆弧后立即结束
                 * 外部会话，避免 M30 及后续空闲周期产生的静止采样污染拟合结果。 */
                if (_circleArcBlockCount >= 2) {
                    finish(CNC_SERVO_STATE_COMPLETED, CNC_SERVO_STOP_PROGRAM_END,
                           CNC_SERVO_ERROR_NONE);
                } else if (programFinished()) {
                    finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_PROGRAM_END,
                           CNC_SERVO_ERROR_CIRCLE_NOT_REACHED);
                }
                return;
            }
            const int32_t lineNumber = cycle_tp->plan_block->line_number;
            if (lineNumber != _circleArcLineNumber) {
                _circleArcLineNumber = lineNumber;
                ++_circleArcBlockCount;
            }
        }
        if (isCompositeConfig(_config)) {
            _actualCompositeVelocityMmPerMin = actualCompositeVelocityMmPerMin();
        } else {
            const double actualPosition = rt_data->axis_data[_config.axis].motor_act_pos;
            _actualVelocityMmPerMin = actualVelocityMmPerMin(actualPosition);
        }
        sampleIfDue();

        if (isRigidTapConfig(_config)) {
            const double rpm = static_cast<double>(spindle_driver.actual_vel);
            const double feed = rt_data->axis_data[_config.axis].act_vel * TIME_MINUS_2_S;
            const double feedCommand = rt_data->axis_data[_config.axis].cmd_vel * TIME_MINUS_2_S;
            const double feedDirection = std::fabs(feedCommand) > kMinVelocity
                ? (feedCommand < 0.0 ? -1.0 : 1.0) : 1.0;
            const double expected = feedDirection * std::fabs(rpm * _config.tapPitchMm);
            // 启动、暂停和换向时转速为零是正常现象。只有确认进给轴确实运动后，
            // 才判定为缺少主轴反馈。
            if (_tapMotionActive && _tapSyncSampleCount > 10U &&
                std::fabs(feed) > 1.0 && std::fabs(rpm) < 1.0e-6) {
                finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_SAFETY,
                       CNC_SERVO_ERROR_TAP_NO_SPINDLE_FEEDBACK);
                return;
            }
            const bool spindleReverse = tapMotion && cycle_tp->plan_block->rigid_data.reverse;
            if (_tapMotionActive && _tapSyncSampleCount > 10U && std::fabs(rpm) > kMinVelocity &&
                ((spindleReverse && rpm > 0.0) || (!spindleReverse && rpm < 0.0))) {
                finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_SAFETY,
                       CNC_SERVO_ERROR_TAP_DIRECTION);
                return;
            }
            if (_tapMotionActive && _tapSyncSampleCount > 10U && std::fabs(feed) > kMinVelocity &&
                    std::fabs(expected - feed) >
                    std::max(1.0, std::fabs(_config.speedMmPerMin) * 0.75)) {
                finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_SAFETY,
                       CNC_SERVO_ERROR_TAP_SYNC);
                return;
            }
        }

        /* 外部会话不使用通用的程序状态结束条件；圆度测试由上面的两段圆弧
         * 采样门控逻辑负责结束。 */
        if (isExternalMotionConfig(_config)) {
            return;
        }

        const bool motionObserved = isCompositeConfig(_config)
            ? std::fabs(var_chan[0][VAR_CHAN_CMD_FEEDRATE].value.f) > kMinVelocity ||
              std::fabs(_actualCompositeVelocityMmPerMin) > kMinVelocity
            : std::fabs(rt_data->axis_data[_config.axis].cmd_vel) > kMinVelocity ||
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
        if (isExternalMotionConfig(_config) && !_circleCaptureStarted) {
            _drainRequested = 0;
            finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_MANUAL,
                   CNC_SERVO_ERROR_CIRCLE_NOT_REACHED);
            return;
        }
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
        if (!_configured) {
            finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_UNREGISTERED,
                   CNC_SERVO_ERROR_PROGRAM_NOT_READY);
        } else if (state == CNC_SERVO_STATE_RUNNING || state == CNC_SERVO_STATE_DRAINING) {
            __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_BUSY, __ATOMIC_RELEASE);
        } else if (isExternalMotionConfig(_config)) {
            // 圆度测试先进入待命状态，避免预定位移动进入采样窗口。
            // 刚性攻丝沿用普通的采样开始流程，在 RUNNING 状态等待攻丝程序块。
            __atomic_store_n(&_shared->state, CNC_SERVO_STATE_ARMED, __ATOMIC_RELEASE);
            __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
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
            // 等待中的最后一个采样点会完成停止请求。
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
    // 复位会清除上一次采样，并将所有会话恢复到注册/程序就绪状态。
    // 外部圆度或攻丝采样只有在 HMI 随后明确发出开始请求后才进入待命状态。
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
    __atomic_store_n(&_shared->testType, _config.testType, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->samplePhase, CNC_SERVO_SAMPLE_PHASE_IDLE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->tapErrorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
    _configured = true;
    _programObserved = false;
    _circleCaptureStarted = false;
    _tapMotionActive = false;
    _circleArcBlockCount = 0;
    _circleArcLineNumber = -1;
    _drainRequested = 0;
    resetCaptureData();
    __atomic_store_n(&_shared->effectivePeriodTicks, _effectiveTicks, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->effectivePeriodNs,
                     static_cast<uint64_t>(_effectiveTicks) * cycleUs * 1000ULL,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->stopReason, CNC_SERVO_STOP_NONE, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE, __ATOMIC_RELEASE);
    // 注册只负责写入配置。NC 文件由 HMI 的常规流程加载；明确的开始请求
    // 才会使外部圆度或攻丝会话进入待命状态。
    __atomic_store_n(&_shared->state,
                     CNC_SERVO_STATE_WAITING_PROGRAM,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
    return true;
}

void ServoSampleSampler::acknowledgeProgram(uint32_t sequence)
{
    if (isExternalMotionConfig(_config)) {
        /* 外部模式不会切换目标模式到 MDI，也不需要程序序列握手。 */
        __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_NONE,
                         __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->acknowledgedSequence, sequence,
                         __ATOMIC_RELEASE);
        return;
    }
    const uint32_t requested = __atomic_load_n(&_shared->requestedProgramSequence,
                                               __ATOMIC_ACQUIRE);
    const uint32_t resetSequence = __atomic_load_n(&_shared->resetSequence,
                                                   __ATOMIC_ACQUIRE);
    const uint32_t state = loadState(_shared);
    if (!_configured || !rt_data || (state != CNC_SERVO_STATE_WAITING_PROGRAM &&
                                     state != CNC_SERVO_STATE_ARMED) ||
        (requested != 0U && requested != resetSequence)) {
        __atomic_store_n(&_shared->errorCode, CNC_SERVO_ERROR_PROGRAM_NOT_READY,
                         __ATOMIC_RELEASE);
        __atomic_store_n(&_shared->acknowledgedSequence, sequence, __ATOMIC_RELEASE);
        return;
    }

    // 保留操作员选择的模式。AUTO、MDI 和手动运动都可以向采样器提供数据，
    // 具体模式由运动控制器解释。
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
    _circleCaptureStarted = false;
    _tapMotionActive = false;
    _circleArcBlockCount = 0;
    _circleArcLineNumber = -1;
    _hasLastActualPosition = false;
    _actualVelocityMmPerMin = 0.0;
    _hasLastCompositePosition = false;
    _actualCompositeVelocityMmPerMin = 0.0;
    _tapSyncIntegralMm = 0.0;
    _tapSyncErrorSumMm = 0.0;
    _tapSyncMinimumMm = 0.0;
    _tapSyncMaximumMm = 0.0;
    _tapSyncSampleCount = 0;
    _hasLastTapFeedback = false;
    _lastTapSpindleAngle = 0.0;
    _lastTapAxisPosition = 0.0;
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
    _circleCaptureStarted = false;
    _tapMotionActive = false;
    _circleArcBlockCount = 0;
    _circleArcLineNumber = -1;
    _hasLastActualPosition = false;
    _actualVelocityMmPerMin = 0.0;
    _hasLastCompositePosition = false;
    _actualCompositeVelocityMmPerMin = 0.0;
    _tapSyncIntegralMm = 0.0;
    _tapSyncErrorSumMm = 0.0;
    _tapSyncMinimumMm = 0.0;
    _tapSyncMaximumMm = 0.0;
    _tapSyncSampleCount = 0;
    _hasLastTapFeedback = false;
    _lastTapSpindleAngle = 0.0;
    _lastTapAxisPosition = 0.0;
    _shared->tapSyncMaxMm = 0.0;
    _shared->tapSyncMinMm = 0.0;
    _shared->tapSyncAverageMm = 0.0;
    __atomic_store_n(&_shared->tapSyncSampleCount, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->samplePhase, CNC_SERVO_SAMPLE_PHASE_IDLE, __ATOMIC_RELEASE);
    _shared->originPositionMm = 0.0;
    __atomic_store_n(&_shared->sampleCount, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->droppedCount, 0U, __ATOMIC_RELEASE);
}

bool ServoSampleSampler::validateConfig(const CncServoSampleConfig& config,
                                        int32_t* error) const
{
    if (config.mode != CNC_SERVO_SAMPLE_MODE_PROGRAM &&
        config.mode != CNC_SERVO_SAMPLE_MODE_EXTERNAL) {
        if (error) *error = CNC_SERVO_ERROR_INVALID_CONFIG;
        return false;
    }
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
            channel.signalId > CNC_SERVO_SIGNAL_RIGID_POS_SYNC_ERROR) {
            if (error) *error = CNC_SERVO_ERROR_INVALID_CONFIG;
            return false;
        }
        if (channel.signalId == CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL ||
            channel.signalId == CNC_SERVO_SIGNAL_ACT_COMPOSITE_VEL) {
            hasCompositeSignal = true;
        } else {
            hasAxisSignal = true;
        }
        if (channel.axis >= SYS_AXES_NUM && channel.axis != 0U) {
            if (error) *error = CNC_SERVO_ERROR_INVALID_AXIS;
            return false;
        }
        validatedMask |= (1U << i);
    }
    if (validatedMask == 0 || validatedMask != config.channelMask) {
        if (error) *error = CNC_SERVO_ERROR_NO_CHANNEL;
        return false;
    }
    if (config.mode == CNC_SERVO_SAMPLE_MODE_EXTERNAL) {
        /* 圆度/外部会话必须包含四个位置通道，并为每个通道指定横轴/纵轴。 */
        if (config.loopType != CNC_SERVO_LOOP_POSITION || hasCompositeSignal ||
            config.channelMask != 0x0FU ||
            config.channels[0].signalId != CNC_SERVO_SIGNAL_ACT_POS ||
            config.channels[1].signalId != CNC_SERVO_SIGNAL_ACT_POS ||
            config.channels[2].signalId != CNC_SERVO_SIGNAL_CMD_POS ||
            config.channels[3].signalId != CNC_SERVO_SIGNAL_CMD_POS ||
            config.channels[0].axis >= SYS_AXES_NUM ||
            config.channels[1].axis >= SYS_AXES_NUM ||
            config.channels[2].axis >= SYS_AXES_NUM ||
            config.channels[3].axis >= SYS_AXES_NUM ||
            config.channels[0].axis == config.channels[1].axis ||
            config.channels[2].axis != config.channels[0].axis ||
            config.channels[3].axis != config.channels[1].axis) {
            if (error) *error = CNC_SERVO_ERROR_INVALID_CONFIG;
            return false;
        }
    } else if (config.testType == CNC_SERVO_TEST_RIGID_TAP) {
        if (config.tapPitchMm <= 0.0 || !std::isfinite(config.tapPitchMm)) {
            if (error) *error = CNC_SERVO_ERROR_TAP_PITCH;
            return false;
        }
        if (config.loopType != CNC_SERVO_LOOP_VELOCITY || config.tapSyncSource != CNC_SERVO_SYNC_SOURCE_SPINDLE ||
            config.channelMask != 0x03U || config.channels[0].signalId != CNC_SERVO_SIGNAL_RIGID_POS_SYNC_ERROR ||
            config.channels[1].signalId != CNC_SERVO_SIGNAL_ACT_VEL || config.channels[0].axis != AXIS_Z ||
            config.channels[1].axis != AXIS_Z || config.tapPlane > 2U ||
            config.tapCycle > 1U || config.tapRPointMm < 0.0 || config.tapDwellMs < 0.0) {
            if (error) *error = CNC_SERVO_ERROR_INVALID_CONFIG;
            return false;
        }
    } else if ((hasCompositeSignal && hasAxisSignal) ||
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
    if (isExternalMotionConfig(config)) {
        for (uint32_t i = 0; i < CNC_SERVO_SAMPLE_CHANNEL_COUNT; ++i) {
            if (!config.channels[i].enabled) continue;
            const uint32_t axis = config.channels[i].axis;
            if (axis >= SYS_AXES_NUM || rt_data->axis_data[axis].status.is_disable) {
                return false;
            }
        }
        return true;
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
            (config.channels[i].signalId == CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL ||
             config.channels[i].signalId == CNC_SERVO_SIGNAL_ACT_COMPOSITE_VEL)) {
            return true;
        }
    }
    return false;
}

bool ServoSampleSampler::isExternalMotionConfig(const CncServoSampleConfig& config) const
{
    return config.mode == CNC_SERVO_SAMPLE_MODE_EXTERNAL;
}

bool ServoSampleSampler::isRigidTapConfig(const CncServoSampleConfig& config) const
{
    return config.testType == CNC_SERVO_TEST_RIGID_TAP;
}

bool ServoSampleSampler::isSpindleSignal(uint32_t signalId) const
{
    return signalId == CNC_SERVO_SIGNAL_ACT_SPINDLE_RPM ||
           signalId == CNC_SERVO_SIGNAL_ACT_SPINDLE_ANGLE ||
           signalId == CNC_SERVO_SIGNAL_CMD_SPINDLE_ANGLE;
}

bool ServoSampleSampler::isCircleMotionActive() const
{
    if (!cycle_tp || !cycle_tp->plan_block) return false;
    return cycle_tp->plan_block->type == CIRCLE_CW ||
           cycle_tp->plan_block->type == CIRCLE_CCW;
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
    const double cmdCompositeVel = var_chan[0][VAR_CHAN_CMD_FEEDRATE].value.f;
    for (uint32_t i = 0; i < CNC_SERVO_SAMPLE_CHANNEL_COUNT; ++i) {
        if (!_config.channels[i].enabled) continue;
        // 通道可以覆盖会话轴。轴 0 同时是会话 X 轴的兼容值，因此既有单轴配置
        // 保持不变，而双轴攻丝会话可以明确采集同步轴。
        const uint32_t axis = _config.channels[i].axis;
        const bool spindleSignal = isSpindleSignal(_config.channels[i].signalId);
        const double channelCmdPos = spindleSignal ? rt_data->axis_data[AXIS_SP].motor_cmd_pos
                                                    : rt_data->axis_data[axis].mac_cmd_pos;
        const double channelActPos = spindleSignal ? rt_data->axis_data[AXIS_SP].motor_act_pos
                                                    : rt_data->axis_data[axis].motor_act_pos;
        // 直接采集实时值，避免依赖共享变量镜像造成一个周期的延迟，
        // 并使调试反馈与实时运动控制器使用相同的数据源。
        const double channelCmdVel = rt_data->axis_data[axis].cmd_vel * TIME_MINUS_2_S;
        const double channelActVel = rt_data->axis_data[axis].act_vel * TIME_MINUS_2_S;
        const double channelFollowErrUm = (channelCmdPos - channelActPos) * 1000.0;
        switch (_config.channels[i].signalId) {
        case CNC_SERVO_SIGNAL_CMD_POS: frame.values[i] = channelCmdPos; break;
        case CNC_SERVO_SIGNAL_ACT_POS: frame.values[i] = channelActPos; break;
        case CNC_SERVO_SIGNAL_FOLLOW_ERR: frame.values[i] = channelFollowErrUm; break;
        case CNC_SERVO_SIGNAL_CMD_VEL: frame.values[i] = channelCmdVel; break;
        case CNC_SERVO_SIGNAL_ACT_VEL: frame.values[i] = channelActVel; break;
        case CNC_SERVO_SIGNAL_CMD_COMPOSITE_VEL: frame.values[i] = cmdCompositeVel; break;
        case CNC_SERVO_SIGNAL_ACT_COMPOSITE_VEL:
            frame.values[i] = _actualCompositeVelocityMmPerMin;
            break;
        case CNC_SERVO_SIGNAL_ACT_SPINDLE_RPM:
            frame.values[i] = static_cast<double>(spindle_driver.actual_vel);
            break;
        case CNC_SERVO_SIGNAL_ACT_SPINDLE_ANGLE:
            frame.values[i] = rt_data->axis_data[AXIS_SP].motor_act_pos;
            break;
        case CNC_SERVO_SIGNAL_CMD_SPINDLE_ANGLE:
            frame.values[i] = rt_data->axis_data[AXIS_SP].motor_cmd_pos;
            break;
        case CNC_SERVO_SIGNAL_RIGID_POS_SYNC_ERROR:
            frame.values[i] = (_tapMotionActive && _config.tapPitchMm > 0.0)
                ? cal_rigid_pos_sync_error() * _config.tapPitchMm * 1000.0 : 0.0;
            break;
        default: break;
        }
    }
    _shared->samples[count] = frame;
    __atomic_store_n(&_shared->sampleCount, count + 1U, __ATOMIC_RELEASE);
    if (isRigidTapConfig(_config) && _tapMotionActive) {
        const double syncErrorMm = cal_rigid_pos_sync_error() * _config.tapPitchMm;
        if (_tapSyncSampleCount == 0) {
            _tapSyncMinimumMm = _tapSyncMaximumMm = syncErrorMm;
        } else {
            _tapSyncMinimumMm = std::min(_tapSyncMinimumMm, syncErrorMm);
            _tapSyncMaximumMm = std::max(_tapSyncMaximumMm, syncErrorMm);
        }
        _tapSyncErrorSumMm += syncErrorMm;
        ++_tapSyncSampleCount;
        _shared->tapSyncMaxMm = std::max(std::fabs(_tapSyncMinimumMm),
                                         std::fabs(_tapSyncMaximumMm));
        _shared->tapSyncMinMm = _tapSyncMinimumMm;
        _shared->tapSyncAverageMm = _tapSyncErrorSumMm /
            static_cast<double>(_tapSyncSampleCount);
        __atomic_store_n(&_shared->tapSyncSampleCount, _tapSyncSampleCount, __ATOMIC_RELEASE);
    }
    /* 安全检查始终基于直接反馈，与上面选择的显示通道无关。 */
    if (isRigidTapConfig(_config) && _tapMotionActive) {
        const double rpm = static_cast<double>(spindle_driver.actual_vel);
        const double feed = rt_data->axis_data[_config.axis].act_vel * TIME_MINUS_2_S;
        const double feedCommand = rt_data->axis_data[_config.axis].cmd_vel * TIME_MINUS_2_S;
        const double feedDirection = std::fabs(feedCommand) > kMinVelocity
            ? (feedCommand < 0.0 ? -1.0 : 1.0) : 1.0;
        const double spindleAngle = rt_data->axis_data[AXIS_SP].motor_act_pos;
        const double tapAxisPosition = rt_data->axis_data[_config.axis].motor_act_pos;
        if (!std::isfinite(spindleAngle) || !std::isfinite(tapAxisPosition) ||
            !std::isfinite(rpm) || !std::isfinite(feed)) {
            finish(CNC_SERVO_STATE_ERROR, CNC_SERVO_STOP_SAFETY,
                   CNC_SERVO_ERROR_TAP_NO_SPINDLE_FEEDBACK);
            return;
        }
        if (!_hasLastTapFeedback) {
            _hasLastTapFeedback = true;
            _lastTapSpindleAngle = spindleAngle;
            _lastTapAxisPosition = tapAxisPosition;
        } else {
            // 刚性攻丝误差与测得的主轴相位相关，而不是与指令转速相关。
            // 对单个采样点的角度增量归一化，避免编码器回绕造成虚假的整圈跳变。
            double angleDelta = spindleAngle - _lastTapSpindleAngle;
            while (angleDelta > 180.0) angleDelta -= 360.0;
            while (angleDelta < -180.0) angleDelta += 360.0;
            const double expectedDelta = feedDirection * std::fabs(angleDelta) / 360.0 *
                                         _config.tapPitchMm;
            const double actualDelta = tapAxisPosition - _lastTapAxisPosition;
            _tapSyncIntegralMm += expectedDelta - actualDelta;
            _lastTapSpindleAngle = spindleAngle;
            _lastTapAxisPosition = tapAxisPosition;
        }
    }
}

void ServoSampleSampler::finish(uint32_t state, uint32_t reason, int32_t error)
{
    __atomic_store_n(&_shared->stopReason, reason, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->errorCode, error, __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->tapErrorCode,
                     isRigidTapConfig(_config) ? error : CNC_SERVO_ERROR_NONE,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->samplePhase,
                     state == CNC_SERVO_STATE_ERROR ? CNC_SERVO_SAMPLE_PHASE_ERROR
                                                     : CNC_SERVO_SAMPLE_PHASE_COMPLETE,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&_shared->state, state, __ATOMIC_RELEASE);
    _programObserved = false;
    _circleCaptureStarted = false;
    _tapMotionActive = false;
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
