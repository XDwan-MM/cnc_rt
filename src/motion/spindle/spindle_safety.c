/**
 * @file spindle_safety.c
 * @brief 主轴停止条件、复位启动锁和受控停车。
 *
 * 安全状态拥有最高控制优先级。急停、驱动器 Fault 或非 HMI 复位立即
 * 撤销力矩意愿；HMI 复位允许 CSV 斜坡停车，刚性攻丝期间则由插补继续
 * 下发 CSP 同步位置，确认主轴与进给轴同时停稳后再撤销使能。
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
#define SPINDLE_RESET_STOP_TIMEOUT_CYCLES  (30000U)
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
    if (rt_data->sys_state[SYS_IS_ALARMING])
    {
        stop_required = true;
    }
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

void spindle_internal_update_reset_stop(void)
{
    HalCAxisStatus axis;
    HalCAxisStatus feed;
    const bool csp = g_spindle.reset_stop_csp;
    const int mode = spindle_hal_mode();
    const int status_rc = rt_hal_axis_status(RT_HAL_AXIS_SP, &axis);
    const int feed_rc = csp ?
        rt_hal_axis_status((RtHalAxisSlot)g_spindle.reset_stop_feed_axis, &feed) : HAL_OK;
    if (++g_spindle.reset_stop_cycles >= SPINDLE_RESET_STOP_TIMEOUT_CYCLES ||
        status_rc != HAL_OK || !axis.speed_valid || !axis.enabled ||
        mode != (csp ? HAL_SPINDLE_CSP : HAL_SPINDLE_CSV) ||
        (csp && (!axis.position_valid || feed_rc != HAL_OK ||
                 !feed.position_valid || !feed.linear_speed_valid)))
    {
        LOG(LOG_LEVEL_ERR, "HMI复位主轴受控停车失败：CSP=%d mode=%d status_rc=%d feed_rc=%d cycles=%u",
            csp, mode, status_rc, feed_rc, g_spindle.reset_stop_cycles);
        g_spindle.reset_stop_pending = false;
        spindle_hal_release_torque();
        setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_RESET_STOP_TIMEOUT);
        return;
    }

    if (csp)
    {
        const double configured_tol = rt_data->axis_data[g_spindle.reset_stop_feed_axis].pos_tol;
        const double feed_tol = isfinite(configured_tol) && configured_tol > 0.0 ?
            configured_tol : 0.01;
        if (cycle_tp != NULL && cycle_tp->exec_state == EXEC_IDLE &&
            fabs(axis.actual_speed_rpm) <= SPINDLE_ACTUAL_ZERO_EPSILON &&
            fabs(axis.command_pos - axis.actual_pos) <= SPINDLE_CSP_POSITION_EPSILON &&
            fabs(feed.actual_speed_mm_min) <= 1.0 &&
            fabs(feed.command_pos - feed.actual_pos) <= feed_tol)
            ++g_spindle.reset_stop_stable_cycles;
        else
            g_spindle.reset_stop_stable_cycles = 0;
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
        LOG(LOG_LEVEL_INFO, "HMI复位：主轴已停稳，撤销使能");
    }
    else
    {
        spindle_hal_request_torque();
    }
}
