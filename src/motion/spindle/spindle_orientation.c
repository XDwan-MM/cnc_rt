#include "rt_drive_control.h"
#include "alarm.h"
/* M19 业务编排；硬件完成判定与预算归 HAL。 */
#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#define SPINDLE_ALARM_ORIENT_TIMEOUT 2100
#define SPINDLE_ORIENT_TIMEOUT_CYCLES 10000U
static void spindle_orientation_apply_deferred_start(void);
void spindle_orientation_failed(bool retain_hold) {
    g_spindle.m19_active=false;
    g_spindle.orient_request_pending=g_spindle.orient_submitted=false;
    g_spindle.orient_exit_submitted=false;
    g_spindle.deferred_start_pending=g_spindle.start_request_pending=false;
    g_spindle.reset_requires_start=true;g_spindle.orient_hold_stop=true;
    g_spindle.run_authorized=g_spindle.program_run_authorized=false;
    g_spindle.orient_phase=retain_hold?SPINDLE_ORIENT_DONE:SPINDLE_ORIENT_IDLE;
    spindle_internal_clear_velocity_arrival();
    if(!retain_hold)(void)alarm_spindle_request_torque(false);
    setAlarm(ALARM_AX,ALARM_ERROR,SPINDLE_ALARM_ORIENT_TIMEOUT);
}
bool spindle_orientation_submit(void) {
    if(g_spindle.orient_submitted)return true;
    int rc=rt_hal_spindle_request_orient(RT_HAL_AXIS_SP);
    if(rc)return false;
    g_spindle.orient_submitted=true;
    return true;
}
bool spindle_orientation_hal_is_done(void) {
    HalAxisSnapshot axis;
    return spindle_hal_snapshot(&axis)==HAL_OK &&
        (axis.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE))==
        (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE) &&
        axis.drive_state==HAL_DRIVE_OPERATION_ENABLED &&
        axis.actual_mode==axis.target_mode && axis.control_state==HAL_CONTROL_ORIENT_HOLD;
}
bool spindle_orientation_release(void) {
    if(g_spindle.orient_exit_submitted)return true;
    HalAxisSnapshot axis;
    if(spindle_hal_snapshot(&axis)!=HAL_OK)return false;
    if(axis.control_state!=HAL_CONTROL_ORIENT_HOLD && axis.control_state!=HAL_CONTROL_ORIENTING)
        return axis.control_state!=HAL_CONTROL_EXITING_ORIENT &&
               axis.control_state!=HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN;
    HalOrientExitTarget target={0};
    if(!(axis.feedback_valid&HAL_FEEDBACK_MODE))return false;
    if(axis.actual_mode==HAL_AXIS_MODE_CSV) {
        target.kind=HAL_ORIENT_EXIT_SPINDLE_SPEED;target.value=0;
    } else if(axis.actual_mode==HAL_AXIS_MODE_CSP &&
              (axis.feedback_valid&HAL_FEEDBACK_POSITION) && isfinite(axis.actual_pos)) {
        target.kind=HAL_ORIENT_EXIT_POSITION;target.value=axis.actual_pos;
    } else return false;
    if(rt_hal_spindle_exit_orient(RT_HAL_AXIS_SP,&target)!=HAL_OK)return false;
    g_spindle.orient_exit_submitted=true;
    return true;
}
void spindle_orientation_abort(void) {
    /* 取消属于停车业务，绝不走那条不支持的正常交接路径。 */
    g_spindle.orient_request_pending=g_spindle.orient_submitted=false;
    g_spindle.orient_exit_submitted=false;
    g_spindle.deferred_start_pending=false;
    g_spindle.m19_active=false;
    g_spindle.orient_phase=SPINDLE_ORIENT_IDLE;
    g_spindle.orient_elapsed_cycles=0;
    (void)alarm_spindle_request_torque(false);
}
void spindle_begin_m19(void)
{
    spindle_internal_clear_velocity_arrival();
    g_spindle.m19_active = true;
    g_spindle.orient_hold_stop = true;
    g_spindle.command_speed = 0.0;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.run_authorized = false;
    g_spindle.program_run_authorized = false;
    g_spindle.start_request_pending = false;
    g_spindle.deferred_start_pending = false;
}

void spindle_request_orientation(bool active)
{
    spindle_internal_clear_velocity_arrival();
    /* 定向接管主轴控制权，不能保留普通 M3/M4 授权。 */
    g_spindle.run_authorized = false;
    g_spindle.program_run_authorized = false;
    /* 只有"开始定向"才取消排队的启动请求；"释放定向"必须保留：
     * 定向期间收到的 M3/M4 存在 deferred_start 里，等释放完成后执行。
     * 以前两条分支都清，导致 M19 完成后同一扫描里按下的 M4 被丢掉
     * （现场表现为定向后主轴反转不转、随后主轴被去使能）。 */
    if (active)
    {
        g_spindle.start_request_pending = false;
        g_spindle.deferred_start_pending = false;
    }
    g_spindle.pending_event = active ?
        SPINDLE_EVENT_ORIENT : SPINDLE_EVENT_RELEASE_ORIENT;
}

bool spindle_orientation_is_done(void)
{
    if (!reset_spindle_business_allowed() ||
        spindle_internal_stop_is_required(&g_spindle) || g_spindle.m29_active)
    {
        return false;
    }
    return spindle_orientation_hal_is_done();
}


void spindle_internal_update_orientation(void) {
    if(g_spindle.orient_phase==SPINDLE_ORIENT_IDLE && !g_spindle.orient_request_pending)return;
    HalAxisSnapshot axis;
    if(spindle_hal_snapshot(&axis)!=HAL_OK || !(axis.feedback_valid&HAL_FEEDBACK_STATUS)) {
        spindle_orientation_failed(false);return;
    }
    if(g_spindle.orient_exit_submitted) {
        if((axis.request_rejection.category==HAL_ISSUE_EXIT_ORIENT && axis.request_rejection.code) ||
           (axis.execution_error.category==HAL_ISSUE_EXIT_ORIENT && axis.execution_error.code)) {
            /* H1 下的 UNSUPPORTED 是明确拒绝，既不是成功释放，也不是自动停车。 */
            spindle_orientation_failed(axis.control_state==HAL_CONTROL_ORIENT_HOLD ||
                               axis.control_state==HAL_CONTROL_ORIENTING);
            return;
        }
        if(axis.control_state!=HAL_CONTROL_ORIENTING && axis.control_state!=HAL_CONTROL_ORIENT_HOLD &&
           axis.control_state!=HAL_CONTROL_EXITING_ORIENT && axis.control_state!=HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN) {
            g_spindle.orient_exit_submitted=false;
            g_spindle.orient_submitted=false;
            g_spindle.orient_phase=SPINDLE_ORIENT_IDLE;
            g_spindle.orient_hold_stop=false;
            spindle_internal_sync_planner_to_actual();
            spindle_orientation_apply_deferred_start();
        }
        return;
    }
    if(g_spindle.orient_request_pending) {
        if(g_spindle.orient_submitted) {
            if(axis.request_rejection.category==HAL_ISSUE_ORIENT && axis.request_rejection.code) {
                spindle_orientation_failed(false);return;
            }
            if(axis.control_state==HAL_CONTROL_ORIENTING||axis.control_state==HAL_CONTROL_ORIENT_HOLD) {
                g_spindle.orient_request_pending=false;
                g_spindle.orient_phase=SPINDLE_ORIENT_RUNNING;
            } else if(axis.execution_error.category==HAL_ISSUE_ORIENT && axis.execution_error.code) {
                spindle_orientation_failed(false);return;
            }
        } else {
            if(spindle_hal_mode_switch_result()!=HAL_OK ||
               (axis.request_rejection.category==HAL_ISSUE_ENABLE&&axis.request_rejection.code)) {
                spindle_orientation_failed(false);return;
            }
            if(spindle_hal_mode_switch_is_done()&&spindle_hal_is_enabled()) {
                if(!spindle_orientation_submit())spindle_orientation_failed(false);
            }
        }
        if(++g_spindle.orient_elapsed_cycles>=SPINDLE_ORIENT_TIMEOUT_CYCLES) {
            spindle_orientation_failed(false);return; /* overall business/preparation watchdog */
        }
    }
    if(g_spindle.orient_phase==SPINDLE_ORIENT_RUNNING) {
        if(spindle_orientation_hal_is_done())g_spindle.orient_phase=SPINDLE_ORIENT_DONE;
        else if(axis.control_state!=HAL_CONTROL_ORIENTING) {
            spindle_orientation_failed(false);return;
        }
    }
    if(g_spindle.orient_phase==SPINDLE_ORIENT_DONE&&g_spindle.m19_active) {
        g_spindle.m19_active=false;spindle_internal_report_mcode_complete();
    }
}
static void spindle_orientation_apply_deferred_start(void)
{
    if (spindle_internal_stop_is_required(&g_spindle) || g_spindle.reset_requires_start)
    {
        g_spindle.deferred_start_pending = false;
        g_spindle.start_request_pending = false;
        return;
    }
    if (g_spindle.m29_active)
    {
        return;
    }

    /* 后到的 CNC/PLC 启动命令优先于先前排队的延迟请求。 */
    if (g_spindle.deferred_start_pending && !g_spindle.start_request_pending)
    {
        g_spindle.command_state = g_spindle.deferred_start_state;
        g_spindle.command_speed = g_spindle.deferred_start_speed;
        g_spindle.start_request_pending = true;
        LOG(LOG_LEVEL_DEBUG, "定向释放：执行定向期间排队的启动，方向=%d 速度=%.0f",
            (int)g_spindle.command_state, g_spindle.command_speed);
    }
    g_spindle.deferred_start_pending = false;

    if (!g_spindle.start_request_pending)
    {
        return;
    }
    if (g_spindle.command_state == SPINDLE_STOP)
    {
        /* 最新命令是停止：不能拿排队的老请求把它顶回去。 */
        g_spindle.start_request_pending = false;
        return;
    }
    spindle_internal_handle_rotation(&g_spindle, g_spindle.command_state);
}
