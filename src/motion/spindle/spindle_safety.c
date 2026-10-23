/**
 * @file spindle_safety.c
 * @brief 主轴停止条件、复位启动锁和受控停车。
 *
 * 安全状态拥有最高控制优先级。急停、驱动器 Fault 或非 HMI 复位立即
 * 撤销力矩意愿；HMI 复位允许 CSV 斜坡停车，刚性攻丝期间则由插补继续
 * 下发 CSP 同步位置，确认主轴与进给轴同时停稳后再撤销使能。
 *
 * "停稳"的判定分轴取可信量：插补空闲（不再下发新的同步位置）+ 主轴
 * 实际转速为零且跟随误差收敛 + 进给轴走到指令位置，连续保持若干拍。
 * 进给轴不看速度——现场 0x606C 在静止点仍有 -9 mm/min 量级的零漂，
 * 而位置差只有 0.0001 mm，用速度当判据会把复位卡到看门狗超时（2103）。
 */

#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include "alarm.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>

#define SPINDLE_SPEED_EPSILON              (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON        (1.0)
#define SPINDLE_CSP_POSITION_EPSILON       (10.0)
#define SPINDLE_STOP_STABLE_CYCLES         (10U)
/* 复位受控停车拆成两段看门狗：
 *   1) 插补/速度斜坡还在减速：保留 30 s 上限。此时机床确实还在动，不能
 *      提前撤力矩（尤其是攻丝还在孔里）；
 *   2) 插补已空闲（CSP 攻丝）：给 10 s 等主轴把 CSP 跟随误差追完。
 *      实测 S100 时冻结位置指令，0.5 s 后仍有 15 rpm、约 1 s 才降到 1 rpm；
 *      原来的 0.5 s 会误杀。10 s 对上万转也只是对数增长，足够且仍有 failsafe。
 * 现场"按复位没反应 30 s"的根因不在时间，而在判据（见下）。
 */
#define SPINDLE_RESET_STOP_TIMEOUT_CYCLES        (30000U)
#define SPINDLE_RESET_STOP_IDLE_TIMEOUT_CYCLES   (10000U)
/* 停车等待期间的诊断打印间隔（拍，1 ms/拍），约 0.5 s 一行，不刷屏 */
#define SPINDLE_RESET_STOP_LOG_CYCLES      (500U)
/* 进给轴位置允差下限（mm）：pos_tol 无效/过小时兜底，用于"已到指令位置"判据 */
#define SPINDLE_RESET_STOP_FEED_POS_FLOOR  (0.01)
#define SPINDLE_ALARM_RESET_STOP_TIMEOUT   (2103)

/* 外部函数定义 */

bool spindle_reset_stop_pending(void)
{
    return g_spindle.reset_stop_pending;
}

bool spindle_reset_stop_uses_csp(void)
{
    return g_spindle.reset_stop_pending && g_spindle.reset_stop_csp;
}

bool spindle_internal_stop_is_required(const spindle_context_t *context)
{
    /* 汇总所有必须停止主轴的原因，供周期入口统一仲裁。 */
    bool stop_required = false;

    if ((spindle_hal_status_word() & 0x0008U) != 0U)
    {
        return true;
    }

    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        stop_required = true;
    }
    if (rt_data->nc_event == SYS_EVENT_HMI_RESET)
    {
        stop_required = true;
    }
    if (nc_reset != NULL && nc_reset->is_resetting)
    {
        /* 复位事件只保持一个 NC 周期，复位流程期间必须持续停主轴。 */
        stop_required = true;
    }
    // if (rt_data->sys_state[SYS_IS_ALARMING])
    // {
    //     stop_required = true;
    // }
    if (context->pending_event == SPINDLE_EVENT_STOP)
    {
        stop_required = true;
    }

    return stop_required;
}

bool spindle_internal_reset_or_estop_active(void)
{
    bool active;

    active = rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
             g_spindle.reset_requires_start ||
             (rt_data->nc_event == SYS_EVENT_HMI_RESET) ||
             (nc_reset != NULL && nc_reset->is_resetting);
    return active;
}

bool spindle_internal_reset_stop_abort_required(void)
{
    return rt_data->sys_state[SYS_ESTOP_INPUT] ||
           rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
           rt_data->sys_state[SYS_IS_ALARMING] ||
           alarm_fault_reset_pending() ||
           rt_data->axis_data[AXIS_SP].status.is_disable ||
           rt_data->axis_data[AXIS_SP].status.is_error ||
           (spindle_hal_status_word() & 0x0008U) != 0U ||
           (nc_reset != NULL && nc_reset->is_resetting &&
            nc_reset->reset_trigger != HMI_RESET);
}

/* 进给轴"已到指令位置"的判据阈值：HMI 定位允差 pos_tol，无效/过小时
 * 用 0.01 mm 兜底。注意这里判的是 HAL 指令位置与反馈位置之差，不判速度：
 * 现场日志里 Z 静止时 0x606C 仍报 -9.3 mm/min，而位置差只有 0.0001 mm，
 * 用速度当判据会永远不成立（30 s 超时的直接原因）。 */
static double spindle_reset_stop_feed_tol(int axis)
{
    double limit;

    if (axis < AXIS_X || axis > AXIS_SP)
        return SPINDLE_RESET_STOP_FEED_POS_FLOOR;

    limit = rt_data->axis_data[axis].pos_tol;
    if (!isfinite(limit) || limit < SPINDLE_RESET_STOP_FEED_POS_FLOOR)
        limit = SPINDLE_RESET_STOP_FEED_POS_FLOOR;
    return limit;
}

/* 把停车判据的每一项都打出来：原来的超时日志只有 CSP/mode/两个返回码/cycles，
 * 现场无法判断到底是"还在转"、"插补没空闲"还是"位置差"卡住。 */
static void spindle_log_reset_stop(int level, const char *what,
                                   const HalCAxisStatus *sp, int sp_rc,
                                   const HalCAxisStatus *feed, int feed_rc,
                                   int mode, bool csp)
{
    const bool sp_ok = (sp != NULL && sp_rc == HAL_OK);
    /* CSV 停车不查进给轴，日志里不要把未读取的零值当成真实反馈 */
    const bool feed_ok = (feed != NULL && feed_rc == HAL_OK && csp);

    LOG(level,
        "%s：CSP=%d mode=%d sp_rc=%d feed_rc=%d cycles=%u idle=%u stable=%u exec=%d "
        "sp_en=%d sp_spd=%.3f sp_cmd=%.3f sp_act=%.3f sp_err=%.3f "
        "feed_en=%d feed_spd=%.3f feed_cmd=%.4f feed_act=%.4f feed_err=%.4f feed_tol=%.4f",
        what, csp ? 1 : 0, mode, sp_rc, feed_rc,
        (unsigned)g_spindle.reset_stop_cycles,
        (unsigned)g_spindle.reset_stop_idle_cycles,
        (unsigned)g_spindle.reset_stop_stable_cycles,
        (cycle_tp != NULL) ? (int)cycle_tp->exec_state : -1,
        sp_ok ? (int)sp->enabled : -1,
        sp_ok ? sp->actual_speed_rpm : 0.0,
        sp_ok ? sp->command_pos : 0.0,
        sp_ok ? sp->actual_pos : 0.0,
        sp_ok ? fabs(sp->command_pos - sp->actual_pos) : 0.0,
        feed_ok ? (int)feed->enabled : -1,
        feed_ok ? feed->actual_speed_mm_min : 0.0,
        feed_ok ? feed->command_pos : 0.0,
        feed_ok ? feed->actual_pos : 0.0,
        feed_ok ? fabs(feed->command_pos - feed->actual_pos) : 0.0,
        spindle_reset_stop_feed_tol(g_spindle.reset_stop_feed_axis));
}

void spindle_internal_update_reset_stop(void)
{
    HalCAxisStatus axis = {0};
    HalCAxisStatus feed = {0};
    const bool csp = g_spindle.reset_stop_csp;
    const int mode = spindle_hal_mode();
    const int status_rc = rt_hal_axis_status(RT_HAL_AXIS_SP, &axis);
    const int feed_rc = csp ?
        rt_hal_axis_status((RtHalAxisSlot)g_spindle.reset_stop_feed_axis, &feed) : HAL_OK;

    /* 两段看门狗：插补/速度斜坡还在走时保留 30 s 上限（机床还在动，
     * 不能提前撤力矩）；CSP 攻丝一旦插补空闲，给 10 s 等主轴把 CSP
     * 跟随误差追完。现场"按复位没反应 30 s"不是时间不够，而是进给轴
     * 速度零漂让判据永远不成立（已改为位置判据，见下）。 */
    const bool interp_idle = (cycle_tp != NULL && cycle_tp->exec_state == EXEC_IDLE);
    ++g_spindle.reset_stop_cycles;
    if (interp_idle)
        ++g_spindle.reset_stop_idle_cycles;
    else
        g_spindle.reset_stop_idle_cycles = 0U;

    /* 读失败/数据不可用/模式不符是"不能继续等"的硬条件，当拍就收口；
     * 是否停稳由下面的分轴判据决定。
     * 只有 CSP 攻丝才用"插补空闲后 10 s"这第二段看门狗：CSV 停车靠
     * 速度规划器自己斜坡，插补空闲（程序复位了）不代表主轴已停，不能用它计时。 */
    if (g_spindle.reset_stop_cycles >= SPINDLE_RESET_STOP_TIMEOUT_CYCLES ||
        (csp && interp_idle &&
         g_spindle.reset_stop_idle_cycles >= SPINDLE_RESET_STOP_IDLE_TIMEOUT_CYCLES) ||
        status_rc != HAL_OK || !axis.speed_valid || !axis.enabled ||
        mode != (csp ? HAL_SPINDLE_CSP : HAL_SPINDLE_CSV) ||
        (csp && (!axis.position_valid || feed_rc != HAL_OK ||
                 !feed.position_valid)))
    {
        spindle_log_reset_stop(LOG_LEVEL_ERR, "HMI复位主轴受控停车失败",
                               &axis, status_rc, &feed, feed_rc, mode, csp);
        g_spindle.reset_stop_pending = false;
        spindle_hal_release_torque();
        setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_RESET_STOP_TIMEOUT);
        return;
    }

    if (csp)
    {
        /*
         * 刚性攻丝复位的"停稳"判据（连续 SPINDLE_STOP_STABLE_CYCLES 拍）：
         *   1) 插补已空闲——不会再有新的进给/主轴同步位置下发；
         *   2) 主轴实际转速为零（主轴的 0x606C 是真实转速，可信）；
         *   3) 主轴跟随误差收敛到 10（用户单位）——跟着速度一起收敛；
         *   4) 进给轴走到指令位置（|command_pos - actual_pos| <= pos_tol）。
         *
         * 进给轴**不再看速度**：实测 Z 静止时 0x606C 仍报 -9.3 mm/min，
         * 而位置差只有 0.0001 mm——速度反馈在静止点的零漂/抖动会让
         * "速度 <= 1 mm/min"永远不成立，这正是现场复位卡 30 s 的原因。
         */
        if (interp_idle &&
            fabs(axis.actual_speed_rpm) <= SPINDLE_ACTUAL_ZERO_EPSILON &&
            fabs(axis.command_pos - axis.actual_pos) <= SPINDLE_CSP_POSITION_EPSILON &&
            fabs(feed.command_pos - feed.actual_pos) <=
                spindle_reset_stop_feed_tol(g_spindle.reset_stop_feed_axis))
        {
            ++g_spindle.reset_stop_stable_cycles;
        }
        else
        {
            g_spindle.reset_stop_stable_cycles = 0U;
            if ((g_spindle.reset_stop_cycles % SPINDLE_RESET_STOP_LOG_CYCLES) == 0U)
                spindle_log_reset_stop(LOG_LEVEL_DEBUG, "HMI复位主轴受控停车等待中",
                                       &axis, status_rc, &feed, feed_rc, mode, csp);
        }
    }
    else
    {
        spindle_internal_update_velocity();
        if (fabs(g_spindle.velocity_planner.current_vel) <= SPINDLE_SPEED_EPSILON &&
            fabs(axis.actual_speed_rpm) <= SPINDLE_ACTUAL_ZERO_EPSILON)
        {
            ++g_spindle.reset_stop_stable_cycles;
        }
        else
        {
            g_spindle.reset_stop_stable_cycles = 0U;
        }
    }

    if (g_spindle.reset_stop_stable_cycles >= SPINDLE_STOP_STABLE_CYCLES)
    {
        g_spindle.reset_stop_pending = false;
        if (csp)
        {
            update_pos_flag = 0;
        }
        spindle_hal_release_torque();
        spindle_log_reset_stop(LOG_LEVEL_INFO, "HMI复位：主轴已停稳，撤销使能",
                               &axis, status_rc, &feed, feed_rc, mode, csp);
    }
    else
    {
        spindle_hal_request_torque();
    }
}
