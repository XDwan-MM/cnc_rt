/** 只负责 M29 的准备：可选的定向/停车、切 CSP、坐标对齐、交接。
 * 位置下发与长期保持归运动模块。 */
#include "alarm.h"
#include "emcmot.h"
#include "motion_csp_stop.h"
#include "rt_drive_control.h"
#include "rt_hal_adapter.h"
#include "rt_hal_config.h"
#include "spindle_control.h"
#include "spindle_internal.h"
#include <hmisysapi.h>
#include <math.h>
#include <regapi.h>

#define M29_ZERO_RPM 1.0
#define M29_POSITION_TOL 10.0
#define M29_STABLE_CYCLES 10U
#define M29_ORIENT_STABLE_CYCLES 50U /* 连续 50 ms 保持稳定 */
#define M29_ORIENT_POSITION_TOL 0.5  /* 稳定窗口及模式交接允差，度；4096ppr 下 0.5°≈5.7 count，覆盖实测 2 count 抖动 */
#define M29_PREP_CYCLES 10000U

/* 内部函数声明 */

static bool read_feedback(HalAxisSnapshot *a);
static void prepare_failed(const char *reason);
static void begin_preparation(spindle_m29_phase_t phase);
static bool start_alignment(void);
static bool orient_position_preserved(const HalAxisSnapshot *a);
static void update_alignment(void);
static void update_csp_preparation(void);

/* 外部函数定义 */

bool spindle_m29_is_active(void) { return g_spindle.m29_active; }
bool spindle_m29_has_error(void) { return g_spindle.m29_phase == SPINDLE_M29_ERROR; }
void spindle_report_m29_prepare_rejected(void) { LOG(LOG_LEVEL_ERR, "M29主轴准备请求被拒绝"); }

bool spindle_request_m29(void)
{
    /* 复位期间拒绝新的业务请求，不能顺手把状态打成 ERROR——那会中断已经在跑的
     * 受控停车。这里只给调用方返回 false。 */
    if (!reset_spindle_business_allowed() || rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
        return false;
    if (spindle_hal_mode() == HAL_AXIS_MODE_CSP)
    {
        const int ready = motion_spindle_prepare_velocity();
        if (ready < 0)
        {
            HalAxisSnapshot a = {0};
            const int rc = spindle_hal_snapshot(&a);
            LOG(LOG_LEVEL_ERR,
                "M29请求被拒：位置控制交接条件不可用 rc=%d valid=0x%x command_valid=0x%x drive=%d mode=%d rpm=%.3f "
                "pos=%.6f target=%.6f",
                rc,
                (unsigned)a.feedback_valid,
                (unsigned)a.command_valid,
                (int)a.drive_state,
                (int)a.actual_mode,
                a.actual_speed_rpm,
                a.actual_pos,
                a.command_pos);
            return false; /* 调用方负责记录拒绝并清理指令。 */
        }
        if (ready == 0)
        {
            LOG(LOG_LEVEL_INFO, "M29请求被拒：位置运动尚未结束或尚未停稳，不排队重试");
            return false;
        }
    }
    begin_preparation(SPINDLE_M29_ORIENTING);
    g_spindle.pending_event = SPINDLE_EVENT_ORIENT;
    return true;
}

bool spindle_request_m29_no_orient(void)
{
    HalAxisSnapshot a;
    int32_t orient;
    const bool switching = spindle_hal_mode_switch_is_pending();
    if (spindle_internal_stop_is_required(&g_spindle) || rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->axis_data[AXIS_SP].status.is_disable || rt_data->axis_data[AXIS_SP].status.is_error ||
        g_spindle.m29_active || g_spindle.orient_phase != SPINDLE_ORIENT_IDLE || g_spindle.orient_request_pending ||
        rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient) != HAL_OK || orient != RT_HAL_ORIENT_NONE ||
        spindle_hal_snapshot(&a) != HAL_OK || !(a.feedback_valid & HAL_FEEDBACK_STATUS) || (a.raw_status & 0x0008U) ||
        ((a.feedback_valid & HAL_FEEDBACK_ERROR_CODE) && a.drive_error_code) ||
        !isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) || rt_data->axis_data[AXIS_SP].max_feed_acc <= 0)
        return false;
    if ((a.feedback_valid & (HAL_FEEDBACK_POSITION | HAL_FEEDBACK_SPEED)) !=
            (HAL_FEEDBACK_POSITION | HAL_FEEDBACK_SPEED) ||
        !isfinite(a.actual_pos) || !isfinite(a.actual_speed_rpm))
        return false;
    const bool mode_known = (a.feedback_valid & HAL_FEEDBACK_MODE) != 0;
    if ((!mode_known || (a.actual_mode != HAL_AXIS_MODE_CSV && a.actual_mode != HAL_AXIS_MODE_CSP)) &&
        fabs(a.actual_speed_rpm) > M29_ZERO_RPM)
        return false;
    if (mode_known && a.actual_mode == HAL_AXIS_MODE_CSP && a.drive_state == HAL_DRIVE_OPERATION_ENABLED)
    {
        if (fabs(a.actual_speed_rpm) > M29_ZERO_RPM ||
            (!g_spindle.reset_requires_start &&
             (!(a.command_valid & HAL_COMMAND_POSITION) || fabs(a.command_pos - a.actual_pos) > M29_POSITION_TOL)))
            return false;
        if (!switching && !g_spindle.csv_switch_pending &&
            rt_hal_request_axis_pos(RT_HAL_AXIS_SP, a.actual_pos) != HAL_OK)
            return false;
    }
    const bool wait_csv = switching || g_spindle.csv_switch_pending || !mode_known ||
                          (a.actual_mode != HAL_AXIS_MODE_CSV && a.actual_mode != HAL_AXIS_MODE_CSP);
    if (wait_csv && spindle_hal_request_mode(HAL_AXIS_MODE_CSV) != HAL_OK)
        return false;
    begin_preparation(wait_csv ? SPINDLE_M29_NO_ORIENT_WAIT_CSV : SPINDLE_M29_NO_ORIENT_STOPPING);
    g_spindle.pending_event = SPINDLE_EVENT_NONE;
    g_spindle.velocity_planner.current_vel = a.actual_speed_rpm;
    g_spindle.velocity_planner.target_vel = 0;
    g_spindle.velocity_planner.enable = false;
    g_spindle_hal.target_position = a.actual_pos;
    if (!wait_csv)
        spindle_hal_mode_switch_reset();
    return true;
}

bool spindle_internal_m29_no_orient_preparing(void)
{
    if (g_spindle.m29_phase == SPINDLE_M29_NO_ORIENT_WAIT_CSV)
    {
        return true;
    }
    if (g_spindle.m29_phase == SPINDLE_M29_NO_ORIENT_STOPPING)
    {
        return true;
    }
    return false;
}
void spindle_internal_update_m29_no_orient(void)
{
    HalAxisSnapshot a;
    int32_t orient;
    if (spindle_internal_stop_is_required(&g_spindle))
    {
        prepare_failed("无预定向准备收到停止请求");
        return;
    }
    if (rt_data->axis_data[AXIS_SP].status.is_disable || rt_data->axis_data[AXIS_SP].status.is_error)
    {
        prepare_failed("无预定向准备主轴禁用或故障");
        return;
    }
    if (spindle_hal_snapshot(&a) != HAL_OK)
    {
        prepare_failed("无预定向准备读取驱动反馈失败");
        return;
    }
    if ((a.feedback_valid & HAL_FEEDBACK_STATUS) == 0 || (a.raw_status & 0x0008U) != 0)
    {
        prepare_failed("无预定向准备驱动状态无效或故障");
        return;
    }
    if ((a.feedback_valid & HAL_FEEDBACK_ERROR_CODE) != 0 && a.drive_error_code != 0)
    {
        LOG(LOG_LEVEL_ERR, "主轴驱动故障：error_code=%d", (int)a.drive_error_code);
        prepare_failed("无预定向准备驱动故障");
        return;
    }
    if (isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) == false ||
        rt_data->axis_data[AXIS_SP].max_feed_acc <= 0)
    {
        prepare_failed("无预定向停车加速度无效");
        return;
    }
    if (rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient) != HAL_OK)
    {
        prepare_failed("无预定向准备读取定向状态失败");
        return;
    }
    if (orient != RT_HAL_ORIENT_NONE)
    {
        prepare_failed("无预定向准备出现定向动作");
        return;
    }
    if (g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
        g_spindle.pending_event == SPINDLE_EVENT_RELEASE_ORIENT)
    {
        prepare_failed("无预定向准备收到定向事件");
        return;
    }
    if (++g_spindle.orient_elapsed_cycles >= M29_PREP_CYCLES)
    {
        prepare_failed("无预定向停车或CSV准备超时");
        return;
    }
    g_spindle.pending_event = SPINDLE_EVENT_NONE;
    (void)alarm_spindle_request_torque(true);
    switch (g_spindle.m29_phase)
    {
    case SPINDLE_M29_NO_ORIENT_WAIT_CSV:
        if (spindle_hal_mode_switch_result() != HAL_OK)
        {
            prepare_failed("CSV模式准备失败");
            return;
        }
        if (!spindle_hal_mode_switch_is_done() || !(a.feedback_valid & HAL_FEEDBACK_MODE) ||
            a.actual_mode != HAL_AXIS_MODE_CSV || a.drive_state != HAL_DRIVE_OPERATION_ENABLED)
            return;
        g_spindle.m29_phase = SPINDLE_M29_NO_ORIENT_STOPPING;
        break;
    case SPINDLE_M29_NO_ORIENT_STOPPING:
        break;
    default:
        return;
    }
    if (!read_feedback(&a))
    {
        prepare_failed("停车反馈失效");
        return;
    }
    const int stopped = a.actual_mode == HAL_AXIS_MODE_CSV ? spindle_prepare_csv_stop_step()
                                                           : (fabs(a.actual_speed_rpm) <= M29_ZERO_RPM ? 1 : 0);
    if (stopped < 0)
    {
        prepare_failed("CSV停车执行失败");
        return;
    }
    if (stopped)
        ++g_spindle.m29_stop_stable_cycles;
    else
        g_spindle.m29_stop_stable_cycles = 0;
    if (g_spindle.m29_stop_stable_cycles < M29_STABLE_CYCLES)
        return;
    /* 等 HAL 侧动作彻底收口再请求切 CSP；否则模式请求会被"控制占用"拒掉。 */
    if (a.control_state != HAL_CONTROL_IDLE)
        return;
    if (spindle_hal_request_mode(HAL_AXIS_MODE_CSP) != HAL_OK)
    {
        prepare_failed("CSP模式请求被拒绝");
        return;
    }
    g_spindle.m29_phase = SPINDLE_M29_SWITCHING_CSP;
    g_spindle.orient_elapsed_cycles = 0;
}
void spindle_internal_update_m29(void)
{
    HalAxisSnapshot feedback;
    int32_t orient;

    if (g_spindle.m29_active == false || spindle_internal_m29_no_orient_preparing() ||
        g_spindle.m29_phase == SPINDLE_M29_READY)
        return;
    switch (g_spindle.m29_phase)
    {
    case SPINDLE_M29_ORIENTING:
        if (g_spindle.orient_phase == SPINDLE_ORIENT_DONE)
        {
            g_spindle.m29_phase = SPINDLE_M29_ORIENT_SETTLING;
            g_spindle.m29_stop_stable_cycles = 0;
            g_spindle.orient_elapsed_cycles = 0;
        }
        else if (g_spindle.orient_phase == SPINDLE_ORIENT_IDLE)
        {
            if (g_spindle.reset_requires_start)
            {
                prepare_failed("定向失败");
                return;
            }
            g_spindle.orient_request_pending = true;
            g_spindle.orient_phase = SPINDLE_ORIENT_WAIT_ZERO;
            g_spindle.orient_elapsed_cycles = 0;
            (void)alarm_spindle_request_torque(true);
        }
        return;
    case SPINDLE_M29_ORIENT_SETTLING:
    {
        /* 保持厂商定向控制字，不以收到 DONE 的瞬间作为模式交接时刻。 */
        HalAxisSnapshot a;
        if (++g_spindle.orient_elapsed_cycles >= M29_PREP_CYCLES)
        {
            prepare_failed("定向后等待稳定超时");
            return;
        }
        if (!read_feedback(&a) || a.drive_state != HAL_DRIVE_OPERATION_ENABLED || a.actual_mode != HAL_AXIS_MODE_CSV ||
            a.control_state != HAL_CONTROL_ORIENT_HOLD || !isfinite(a.coordinate_offset))
        {
            prepare_failed("定向保持状态或反馈失效");
            return;
        }
        if (!(a.raw_status & RT_PLACEHOLDER_ORIENT_DONE) || fabs(a.actual_speed_rpm) > M29_ZERO_RPM)
        {
            g_spindle.m29_stop_stable_cycles = 0;
            return;
        }
        const double position = a.actual_pos - a.coordinate_offset;
        if (!g_spindle.m29_stop_stable_cycles)
        {
            g_spindle.m29_settle_min = g_spindle.m29_settle_max = position;
        }
        /* 比较窗口极差而非相邻拍差，避免缓慢单向漂移被连续小增量放过。 */
        g_spindle.m29_settle_min = fmin(g_spindle.m29_settle_min, position);
        g_spindle.m29_settle_max = fmax(g_spindle.m29_settle_max, position);
        if (g_spindle.m29_settle_max - g_spindle.m29_settle_min > M29_ORIENT_POSITION_TOL)
        {
            g_spindle.m29_stop_stable_cycles = 0;
            return;
        }
        if (++g_spindle.m29_stop_stable_cycles >= M29_ORIENT_STABLE_CYCLES)
        {
            g_spindle.m29_orient_reference = position;
            g_spindle.m29_orient_reference_valid = true;
            /* 真正的 CSV→CSP 模式切换会让驱动器取消定向。H1 不支持同模式下
             * 假装正常释放的做法。 */
            if (spindle_hal_mode() == HAL_AXIS_MODE_CSP || spindle_hal_request_mode(HAL_AXIS_MODE_CSP) != HAL_OK)
            {
                prepare_failed("定向到CSP交接被拒绝");
                return;
            }
            g_spindle.m29_phase = SPINDLE_M29_SWITCHING_CSP;
            g_spindle.orient_phase = SPINDLE_ORIENT_IDLE;
            g_spindle.orient_request_pending = g_spindle.orient_submitted = false;
            g_spindle.orient_exit_submitted = false;
            g_spindle.orient_elapsed_cycles = 0;
            (void)alarm_spindle_request_torque(true);
        }
        return;
    }
    case SPINDLE_M29_SWITCHING_CSP:
    case SPINDLE_M29_ALIGNING:
        if (spindle_internal_stop_is_required(&g_spindle) || rt_data->axis_data[AXIS_SP].status.is_disable ||
            rt_data->axis_data[AXIS_SP].status.is_error)
        {
            prepare_failed("CSP准备中止");
            return;
        }
        if (spindle_hal_snapshot(&feedback) != HAL_OK)
        {
            prepare_failed("CSP准备反馈失效");
            return;
        }
        if ((feedback.feedback_valid & HAL_FEEDBACK_STATUS) == 0 || (feedback.raw_status & 0x0008U) != 0)
        {
            prepare_failed("CSP准备驱动状态异常");
            return;
        }
        if ((feedback.feedback_valid & HAL_FEEDBACK_ERROR_CODE) != 0 && feedback.drive_error_code != 0)
        {
            prepare_failed("CSP准备驱动故障");
            return;
        }
        /* 无定向参考的准备仍须禁止意外定向，并保留原有加速度联锁。
         * 带定向路径的物理位置连续性由后续公共准备函数校验。 */
        if (g_spindle.m29_orient_reference_valid == false)
        {
            if (isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) == false ||
                rt_data->axis_data[AXIS_SP].max_feed_acc <= 0)
            {
                prepare_failed("无预定向准备加速度无效");
                return;
            }
            if (rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient) != HAL_OK || orient != RT_HAL_ORIENT_NONE)
            {
                prepare_failed("无预定向准备出现定向动作");
                return;
            }
            if (g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
                g_spindle.pending_event == SPINDLE_EVENT_RELEASE_ORIENT)
            {
                prepare_failed("无预定向准备收到定向事件");
                return;
            }
        }
        g_spindle.pending_event = SPINDLE_EVENT_NONE;
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0;
        g_spindle_hal.target_velocity = 0;
        (void)alarm_spindle_request_torque(true);
        if (++g_spindle.orient_elapsed_cycles >= M29_PREP_CYCLES)
        {
            prepare_failed("CSP准备超时");
            return;
        }
        if (g_spindle.m29_phase == SPINDLE_M29_SWITCHING_CSP)
            update_csp_preparation();
        else
            update_alignment();
        return;
    default:
        prepare_failed("无效准备阶段");
        return;
    }
}

bool spindle_m29_is_ready(void)
{
    /* 稳定性在对齐阶段锁存；此处不重新累计稳定窗口，也不校验绝对定向角度。 */
    HalAxisSnapshot a;
    return !spindle_internal_stop_is_required(&g_spindle) && g_spindle.m29_active && !g_spindle.reset_requires_start &&
           g_spindle.m29_phase == SPINDLE_M29_READY && g_spindle.m29_feedback_stable &&
           motion_spindle_position_active() && read_feedback(&a) && a.actual_mode == HAL_AXIS_MODE_CSP &&
           a.drive_state == HAL_DRIVE_OPERATION_ENABLED;
}
void spindle_complete_m29(void)
{
    if (!spindle_m29_is_ready())
        return;
    /* 正常握手完成只归还业务状态，不能释放运动模块持有的位置许可和力矩。 */
    g_spindle.m29_active = false;
    g_spindle.m29_phase = SPINDLE_M29_IDLE;
    g_spindle.m29_feedback_stable = false;
    g_spindle.orient_hold_stop = false;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.command_speed = 0;
    g_spindle.manual_run_uses_program_speed = false;
    g_spindle.run_authorized = g_spindle.program_run_authorized = false;
    spindle_internal_clear_velocity_arrival();
}
void spindle_internal_publish_rigid_state(void)
{
    rt_data->sys_state[SYS_IS_RIGID] = motion_spindle_position_active() || motion_csp_stop_pending() ||
                                       (g_spindle.m29_phase != SPINDLE_M29_IDLE &&
                                        g_spindle.m29_phase != SPINDLE_M29_ERROR);
}

/* 内部函数定义 */

static bool read_feedback(HalAxisSnapshot *a)
{
    return spindle_hal_snapshot(a) == HAL_OK &&
           (a->feedback_valid &
            (HAL_FEEDBACK_STATUS | HAL_FEEDBACK_MODE | HAL_FEEDBACK_POSITION | HAL_FEEDBACK_SPEED)) ==
               (HAL_FEEDBACK_STATUS | HAL_FEEDBACK_MODE | HAL_FEEDBACK_POSITION | HAL_FEEDBACK_SPEED) &&
           isfinite(a->actual_pos) && isfinite(a->actual_speed_rpm);
}
static void prepare_failed(const char *reason)
{
    g_spindle.m29_phase = SPINDLE_M29_ERROR;
    g_spindle.m29_active = g_spindle.m29_feedback_stable = false;
    g_spindle.reset_requires_start = true;
    g_spindle.start_request_pending = g_spindle.deferred_start_pending = false;
    g_spindle.command_state = g_spindle.run_state = SPINDLE_STOP;
    g_spindle.command_speed = 0;
    g_spindle.manual_run_uses_program_speed = false;
    g_spindle.velocity_planner.enable = false;
    g_spindle.velocity_planner.target_vel = g_spindle_hal.target_velocity = 0;
    spindle_internal_clear_velocity_arrival();
    spindle_hal_mode_switch_reset();
    motion_spindle_release_position();
    (void)alarm_spindle_request_torque(false);
    LOG(LOG_LEVEL_ERR, "M29准备失败：%s", reason);
}
static void begin_preparation(spindle_m29_phase_t phase)
{
    motion_spindle_release_position();
    spindle_internal_clear_velocity_arrival();
    g_spindle.m29_active = true;
    g_spindle.m29_phase = phase;
    g_spindle.m29_feedback_stable = false;
    g_spindle.m29_orient_reference_valid = false;
    g_spindle.m29_stop_stable_cycles = 0;
    g_spindle.orient_elapsed_cycles = 0;
    g_spindle.reset_requires_start = false;
    g_spindle.m19_active = false;
    g_spindle.orient_hold_stop = true;
    g_spindle.csv_switch_pending = false;
    g_spindle.start_request_pending = g_spindle.deferred_start_pending = false;
    g_spindle.command_state = g_spindle.run_state = SPINDLE_STOP;
    g_spindle.run_authorized = g_spindle.program_run_authorized = false;
    g_spindle.command_speed = 0;
    g_spindle.manual_run_uses_program_speed = false;
}
static bool start_alignment(void)
{
    HalAxisSnapshot a;
    if (!read_feedback(&a))
        return false;
    /* 软件设零不转动主轴；完成以 PDO 线程的执行回执为准。 */
    g_spindle.m29_hold_submitted = false;
    g_spindle.m29_stop_stable_cycles = 0;
    g_spindle.orient_elapsed_cycles = 0;
    return isfinite(a.coordinate_offset) && rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0) == HAL_OK;
}
static bool orient_position_preserved(const HalAxisSnapshot *a)
{
    /* 去掉软件偏置后再比较，避免设零掩盖切模式期间的实际反馈变化。
     * 该式对 set_pos 不变（actual 与 offset 同步平移），因此判失败即真实位置
     * 在 CSV→CSP 交接期间发生了变化；失败时把实际量打出来供现场判读。 */
    const double physical = isfinite(a->coordinate_offset) ? a->actual_pos - a->coordinate_offset : NAN;
    if (!g_spindle.m29_orient_reference_valid)
        return true;
    if (isfinite(physical) && fabs(physical - g_spindle.m29_orient_reference) <= M29_ORIENT_POSITION_TOL)
        return true;
    LOG(LOG_LEVEL_ERR,
        "M29定向位置偏移：actual=%.4f offset=%.4f physical=%.4f ref=%.4f delta=%.4f tol=%.3f rpm=%.3f phase=%d",
        a->actual_pos,
        isfinite(a->coordinate_offset) ? a->coordinate_offset : NAN,
        physical,
        g_spindle.m29_orient_reference,
        physical - g_spindle.m29_orient_reference,
        (double)M29_ORIENT_POSITION_TOL,
        a->actual_speed_rpm,
        (int)g_spindle.m29_phase);
    return false;
}
static void update_alignment(void)
{
    HalAxisSnapshot a;
    const int awaited = g_spindle.m29_hold_submitted ? HAL_ISSUE_POSITION : HAL_ISSUE_SET_POSITION;
    if (!read_feedback(&a) || a.drive_state != HAL_DRIVE_OPERATION_ENABLED || a.actual_mode != HAL_AXIS_MODE_CSP ||
        (a.request_rejection.category == awaited && a.request_rejection.code))
    {
        prepare_failed("坐标对齐反馈或请求异常");
        return;
    }
    if (!orient_position_preserved(&a))
    {
        prepare_failed("定向位置在CSP交接期间发生偏移");
        return;
    }
    if (!g_spindle.m29_hold_submitted)
    {
        int32_t result;
        const int rc = rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP, &result);
        if (rc == HAL_ERROR_BUSY)
            return;
        if (rc != HAL_OK || result != HAL_OK)
        {
            prepare_failed("软件设零未执行成功");
            return;
        }
        g_spindle.m29_expected_offset = a.coordinate_offset;
        /* 不要求反馈在请求后完全不动，仍保留物理位置连续性保护。 */
        if (rt_hal_request_axis_pos(RT_HAL_AXIS_SP, 0) != HAL_OK)
            prepare_failed("保持目标被拒绝");
        else
            g_spindle.m29_hold_submitted = true;
        return;
    }
    if (!(a.command_valid & HAL_COMMAND_POSITION) || fabs(a.command_pos) > 1e-6 ||
        fabs(a.actual_speed_rpm) > M29_ZERO_RPM || fabs(a.command_pos - a.actual_pos) > M29_POSITION_TOL)
    {
        g_spindle.m29_stop_stable_cycles = 0;
        return;
    }
    if (++g_spindle.m29_stop_stable_cycles < M29_STABLE_CYCLES)
        return;
    /* 只授予位置输出许可；READY 仍需等待 PLC FIN，期间维持同一零目标。 */
    if (!motion_spindle_take_position(0))
    {
        prepare_failed("运动模块接管失败");
        return;
    }
    g_spindle_hal.target_position = 0;
    g_spindle.m29_phase = SPINDLE_M29_READY;
    g_spindle.m29_feedback_stable = true;
    setRegBit(REG_F, 45, 1);
}
/* 两条准备路径共用的状态推进：模式就绪 → 显式使能 → 坐标对齐。 */
static void update_csp_preparation(void)
{
    HalAxisSnapshot a;
    if (spindle_hal_mode_switch_result() != HAL_OK)
    {
        prepare_failed("CSP模式准备失败");
        return;
    }
    if (spindle_hal_snapshot(&a) != HAL_OK || !(a.feedback_valid & HAL_FEEDBACK_STATUS))
    {
        prepare_failed("CSP状态失效");
        return;
    }
    /* D32：模式准备 READY 与实际模式反馈是两件事。台达要等独立的使能完成
     * 之后才可能报出实际模式。 */
    if (!spindle_hal_mode_switch_is_done() || a.drive_state != HAL_DRIVE_OPERATION_ENABLED)
        return;
    const bool feedback_valid = read_feedback(&a);
    if (feedback_valid && !orient_position_preserved(&a))
    {
        prepare_failed("定向位置在切CSP后发生偏移");
        return;
    }
    if (!feedback_valid || a.actual_mode != HAL_AXIS_MODE_CSP || fabs(a.actual_speed_rpm) > M29_ZERO_RPM)
        return;
    if (!start_alignment())
    {
        prepare_failed("软件起点设置被拒绝");
        return;
    }
    g_spindle.m29_phase = SPINDLE_M29_ALIGNING;
}
