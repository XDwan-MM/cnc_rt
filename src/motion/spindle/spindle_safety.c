#include "rt_drive_control.h"
/** 主轴安全判据与 CSV 复位减速执行器。
 * CSP 同步停车/保持归 motion_csp_stop.c；公共复位负责协调、最终下使能、
 * 坐标重建与整体收口。 */
#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include "alarm.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#define SPINDLE_SPEED_EPSILON 1e-6
#define SPINDLE_ACTUAL_ZERO_EPSILON 1.0
#define SPINDLE_STOP_STABLE_CYCLES 10U
#define SPINDLE_RESET_STOP_TIMEOUT_CYCLES 30000U
#define SPINDLE_ALARM_RESET_STOP_TIMEOUT 2103

bool spindle_csv_stop_pending(void) { return g_spindle.csv_stop_pending; }
void spindle_csv_stop_cancel(void) { g_spindle.csv_stop_pending=false; }

bool spindle_internal_stop_is_required(const spindle_context_t *context)
{
    /* 汇总所有必须停止主轴的原因，供周期入口统一仲裁。 */
    bool stop_required = false;

    if ((spindle_hal_status_word() & 0x0008U) != 0U)
    {
        return true;
    }

    if (rt_data->sys_state[SYS_ESTOP_INPUT] || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
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
    /* 通用报警不在此一律撤力矩；故障/急停及复位停车由公共流程裁决。 */
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
           alarm_fault_reset_pending() ||
           rt_data->axis_data[AXIS_SP].status.is_disable ||
           rt_data->axis_data[AXIS_SP].status.is_error ||
           (spindle_hal_status_word() & 0x0008U) != 0U ||
           (nc_reset != NULL && nc_reset->is_resetting &&
            nc_reset->reset_trigger != HMI_RESET);
}


void spindle_internal_update_csv_stop(void)
{
    /* 普通业务已被取消，本执行器仅保留减速所需的 CSV 目标及停车反馈。 */
    RtHalAxisStatus axis={0};
    ++g_spindle.csv_stop_cycles;
    if(g_spindle.csv_stop_cycles>=SPINDLE_RESET_STOP_TIMEOUT_CYCLES ||
       rt_hal_axis_status(RT_HAL_AXIS_SP,&axis)!=HAL_OK || !axis.speed_valid || !axis.enabled ||
       spindle_hal_mode()!=HAL_AXIS_MODE_CSV) {
        LOG(LOG_LEVEL_ERR,"CSV复位停车失败：cycles=%u",g_spindle.csv_stop_cycles);
        g_spindle.csv_stop_pending=false;
        reset_stop_executor_finished(false);
        setAlarm(ALARM_AX,ALARM_ERROR,SPINDLE_ALARM_RESET_STOP_TIMEOUT);
        return;
    }
    if(!spindle_internal_update_velocity()) {
        g_spindle.csv_stop_pending=false;
        reset_stop_executor_finished(false);
        setAlarm(ALARM_AX,ALARM_ERROR,SPINDLE_ALARM_RESET_STOP_TIMEOUT);
        return;
    }
    if(fabs(g_spindle.velocity_planner.current_vel)<=SPINDLE_SPEED_EPSILON &&
       fabs(axis.actual_speed_rpm)<=SPINDLE_ACTUAL_ZERO_EPSILON)
        ++g_spindle.csv_stop_stable_cycles;
    else g_spindle.csv_stop_stable_cycles=0;
    if(g_spindle.csv_stop_stable_cycles>=SPINDLE_STOP_STABLE_CYCLES) {
        g_spindle.csv_stop_pending=false;
        reset_stop_executor_finished(true);
        LOG(LOG_LEVEL_INFO,"CSV复位停车完成，交公共流程下使能");
    }
}
void spindle_csv_stop_begin(void)
{
    /* 只接管已经使能且正在运动的 CSV；不能为执行停车重新使能。 */
    if(!g_spindle.csv_stop_pending && !g_spindle.reset_requires_start &&
       !spindle_internal_reset_stop_abort_required() && spindle_hal_is_enabled() &&
       spindle_hal_mode()==HAL_AXIS_MODE_CSV &&
       (fabs(spindle_hal_actual_velocity())>SPINDLE_ACTUAL_ZERO_EPSILON ||
        fabs(g_spindle.velocity_planner.current_vel)>SPINDLE_SPEED_EPSILON)) {
        g_spindle.csv_stop_pending=true;
        g_spindle.csv_stop_cycles=g_spindle.csv_stop_stable_cycles=0;
        LOG(LOG_LEVEL_INFO,"HMI复位：主轴执行CSV减速停车");
    }
}
