#include "rt_drive_control.h"
#include "emcmot.h"
#include "motion_csp_stop.h"
#include "reset.h"
#include "alarm.h"
#include "rt_hal_adapter.h"
#include "rerun.h"
#include "rt_data.h"
#include "spindle_control.h"
#include "tapreturn.h"
#include "global.h"
#include <hmisysapi.h>
#include <stdlib.h>

/* 公共复位只协调顺序；减速计算和设备执行分别留在运动模块与HAL。 */
enum {
    RESET_IDLE,      /* 没有本次复位 */
    RESET_STOPPING,  /* 等待运动结束及必要的受控停车 */
    RESET_WAIT_OFF,  /* 等待主轴实际下使能 */
    RESET_REBUILD,   /* 等待必要的坐标基准重建 */
    RESET_COMPLETE, /* 本次复位完成 */
    RESET_FAILED    /* 本次失败，不能继续按成功收尾 */
};
static int s_phase;
static bool s_prepare_pending;  /* 本次停车尚未选择执行方，仅准备一次 */
static bool s_disable_sent;     /* 下使能请求已成功暂存，仍须等实际反馈 */
static bool s_disable_attempted;/* 已尝试提交，失败后不每拍重试 */
static bool s_rebuild_spindle;  /* 本次CSP停车需要保护并重建主轴基准 */
static bool s_spindle_rebuilt;  /* 本次主轴基准已用有效反馈重建 */

const char* reset_trigger_name[] = {"紧急停止信号ESP", "外部复位信号ERS", "复位&倒带信号RRW", "按下MDI的RESET键"};

GNC_RES reset_data_init(){
    nc_reset = (RESET_DATA*)calloc(sizeof(RESET_DATA), 1);
    if(nc_reset == NULL) return GNC_DATA_NULL;
    motion_spindle_release_position();
    motion_csp_stop_cancel();
    spindle_csv_stop_cancel();
    s_phase = RESET_IDLE;
    s_prepare_pending = s_disable_sent = s_disable_attempted = s_rebuild_spindle = s_spindle_rebuilt = false;
    nc_reset->is_mdi_reset = 0;
    nc_reset->is_resetting = 0;
    nc_reset->is_resetted = 0;
    nc_reset->is_reset_delaying = 0;
    nc_reset->reset_tic = 0;
    nc_reset->mdi_reset_tic = 0;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES reset_data_reset(){
    if(nc_reset == NULL) return GNC_DATA_NULL;
    motion_spindle_release_position();
    motion_csp_stop_cancel();
    spindle_csv_stop_cancel();
    s_phase = RESET_IDLE;
    s_prepare_pending = s_disable_sent = s_disable_attempted = s_rebuild_spindle = s_spindle_rebuilt = false;
    nc_reset->is_mdi_reset = 0;
    nc_reset->is_resetting = 0;
    nc_reset->is_resetted = 0;
    nc_reset->is_reset_delaying = 0;
    nc_reset->reset_tic = 0;
    nc_reset->mdi_reset_tic = 0;
    return GNC_OPERATOR_SUCCESS;
}

/// @brief 触发F1.1的信号来源
/// @param reset_trigger 
void nc_set_resetting(RESET_TRIGGER reset_trigger){
    if (!nc_reset) return;
    /* 同一复位尚在进行时保留停车进度；新的急停/复位来源仍可抢占。 */
    if (nc_reset->is_resetting && nc_reset->reset_trigger == reset_trigger &&
        s_phase != RESET_FAILED && s_phase != RESET_COMPLETE && s_phase != RESET_IDLE) {
        /* 重复按 HMI 复位必须重新给出 F6.1 握手脉冲：PLC 是靠 F6.1 把
         * G8.4 置 1 的。第一次脉冲可能落在急停仍按着、PLC 不接受的时候，
         * 拔起后再按复位如果被这里的早退吞掉，F6.1 就再也不会出现，
         * G8.4 永远为 0。 */
        if (reset_trigger == HMI_RESET) {
            nc_reset->is_mdi_reset = 1;
            nc_reset->mdi_reset_tic = 0;
        }
        return;
    }
    /* 新的一次复位：撤销旧业务，后续在运动计算前选择停车执行方。 */
    motion_spindle_release_position();
    motion_csp_stop_cancel();
    spindle_csv_stop_cancel();
    s_phase = reset_trigger == HMI_RESET ? RESET_STOPPING : RESET_WAIT_OFF;
    s_prepare_pending = true;
    s_disable_sent = s_disable_attempted = s_rebuild_spindle = s_spindle_rebuilt = false;
    nc_reset->is_resetted = 0;
    /* 任何一次新复位都允许重试故障复位：否则收口失败一次后 s_reset_failed
     * 一直把四轴压在未使能态，只能重启软件。 */
    alarm_reset_latch_clear();
    nc_reset->reset_trigger = reset_trigger;
    nc_reset->is_resetting = 1;
   
    LOG(LOG_LEVEL_DEBUG, "复位中F1.1信号来源：%s", reset_trigger_name[reset_trigger]);
    switch (nc_reset->reset_trigger)
    {
    // 基于MDI的复位确认信号
    case HMI_RESET:
        nc_reset->is_mdi_reset = 1;
        nc_reset->mdi_reset_tic = 0;
        break;
    default:
        break;
    }
}

/* 公共 RT 协调入口。硬件推进与超时预算仍留在 HAL 里。 */
void reset_prepare_motion(void)
{
    if (!nc_reset || !nc_reset->is_resetting || s_phase == RESET_FAILED) return;
    /* SYS_IS_ALARMING 汇总所有轴的报警。其他轴的限位报警不应中断主轴受控停车；
     * 急停和故障复位状态仍通过下面的专用条件拦截。 */
    if (rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        alarm_fault_reset_pending()) {
        spindle_csv_stop_cancel();
        motion_csp_stop_cancel();
        if (s_phase != RESET_FAILED) s_phase = RESET_WAIT_OFF;
        s_prepare_pending = false;
        return;
    }
    HalAxisSnapshot status;
    if (rt_hal_axis_snapshot(RT_HAL_AXIS_SP, &status) != HAL_OK ||
        !(status.feedback_valid & HAL_FEEDBACK_STATUS)) {
        spindle_csv_stop_cancel();
        motion_csp_stop_cancel();
        s_phase = RESET_FAILED;
        alarm_report_hal_error(HAL_ERROR_STATE);
        return;
    }
    if (s_prepare_pending) {
        s_prepare_pending = false;
        if (nc_reset->reset_trigger == HMI_RESET) {
            if(!motion_csp_stop_begin())spindle_csv_stop_begin();
            s_rebuild_spindle = motion_csp_stop_pending();
        }
    }
}

bool reset_stop_pending(void)
{
    return spindle_csv_stop_pending() || motion_csp_stop_pending();
}

void reset_stop_executor_finished(bool success)
{
    if (s_phase != RESET_STOPPING) return;
    s_phase = success ? RESET_WAIT_OFF : RESET_FAILED;
}

bool reset_spindle_business_allowed(void)
{
    return (s_phase == RESET_IDLE || s_phase == RESET_COMPLETE) &&
           (!nc_reset || !nc_reset->is_resetting);
}

bool reset_spindle_torque_allowed(bool ordinary)
{
    if (s_phase == RESET_IDLE || s_phase == RESET_COMPLETE) return ordinary;
    /* 受控停车期间需要主轴力矩完成减速，不以其他轴的汇总报警位撤销需求。 */
    return s_phase == RESET_STOPPING && reset_stop_pending() &&
           !rt_data->sys_state[SYS_ESTOP_INPUT] &&
           !rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] &&
           !alarm_fault_reset_pending();
}

bool reset_spindle_disable_needed(void)
{
    return s_phase != RESET_IDLE && s_phase != RESET_COMPLETE &&
           !reset_spindle_torque_allowed(false) && !s_disable_attempted;
}

void reset_spindle_disable_submitted(int rc)
{
    s_disable_attempted = true;
    if (rc == HAL_OK) s_disable_sent = true;
    else {
        s_phase = RESET_FAILED;
        alarm_report_hal_error(rc);
    }
}

bool reset_position_rebuild_allowed(void)
{
    /* 只有正在进行的 CSP 停车才拥有共享坐标基准的处置权；
     * CSV/故障恢复仍沿用原有的进给轴重建策略。 */
    return !s_rebuild_spindle || s_phase == RESET_COMPLETE || s_phase == RESET_REBUILD;
}

bool reset_spindle_position_required(void)
{
    return s_rebuild_spindle && s_phase == RESET_REBUILD && !s_spindle_rebuilt;
}
void reset_spindle_position_rebuilt(void)
{
    if (s_phase == RESET_REBUILD) s_spindle_rebuilt=true;
}

/// @brief 判断复位是否完成：按原加工模式条件等待，再确认下使能及必要重建。
/* 每拍在alarm_poll_hal之后、公共使能提交之前调用，使用本拍已发布反馈。 */
void nc_set_resetted(void)
{
    if (!nc_reset || !nc_reset->is_resetting || s_phase == RESET_IDLE ||
        s_phase == RESET_COMPLETE || s_phase == RESET_FAILED) return;
    /* 普通轴报警可在复位后继续存在；复位完成仍由运动、下使能和轴基准条件裁决。 */
    if (rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] || alarm_fault_reset_pending()) return;

    /* 1. 保留原按加工模式判断运动结束的规则；此处不推进或清理插补。 */
    bool motion_idle;
    switch (rt_data->current_mode) {
    case MODE_MDI: // MDI
    case MODE_MEM: // 记忆
    case MODE_DNC:
        motion_idle = cycle_tp && cycle_tp->exec_state == EXEC_IDLE;
        break;
    case MODE_HOMING: // 回零
        motion_idle = !var_chan[0][VAR_CHAN_IS_HOMING].value.i;
        break;
    case MODE_JOG: // 点动
    case MODE_MPG: // 手轮
    case MODE_IDLE:
    case MODE_EDIT:
    default:
        motion_idle = true;
        break;
    }

    /* 2. 等待受控停车及原零速条件，停车在途不能清运行数据。 */
    if (s_phase == RESET_STOPPING) {
        if (reset_stop_pending() || !motion_idle ||
            !IsEqualTo(rt_hal_spindle_rpm(), 0)) return;
        s_phase = RESET_WAIT_OFF;
        return; /* never complete from the snapshot preceding our disable */
    }
    /* 3. 请求已暂存不等于已下使能；失败/未知状态都不能算完成。 */
    if (s_phase == RESET_WAIT_OFF) {
        if (!s_disable_sent) return;
        HalAxisSnapshot axis;
        int rc = rt_hal_axis_snapshot(RT_HAL_AXIS_SP, &axis);
        if (rc != HAL_OK) {
            s_phase = RESET_FAILED; alarm_report_hal_error(rc); return;
        }
        if ((axis.request_rejection.category == HAL_ISSUE_DISABLE && axis.request_rejection.code) ||
            (axis.execution_error.category == HAL_ISSUE_DISABLE && axis.execution_error.code)) {
            rc = axis.request_rejection.category == HAL_ISSUE_DISABLE && axis.request_rejection.code ?
                 axis.request_rejection.code : axis.execution_error.code;
            s_phase = RESET_FAILED; alarm_report_hal_error(rc); return;
        }
        if (!(axis.feedback_valid & HAL_FEEDBACK_STATUS)) {
            s_phase = RESET_FAILED; alarm_report_hal_error(HAL_ERROR_STATE); return;
        }
        if (axis.control_state != HAL_CONTROL_IDLE ||
            (axis.drive_state != HAL_DRIVE_READY &&
             axis.drive_state != HAL_DRIVE_SWITCH_ON_DISABLED)) return;
        if (!motion_idle || !IsEqualTo(rt_hal_spindle_rpm(), 0)) return;
        s_phase = RESET_REBUILD;
        if (s_rebuild_spindle) update_pos_flag = 0;
        return;
    }
    /* 4. 必要重建完成后，沿原收尾动作统一完成本次复位。 */
    if (s_phase != RESET_REBUILD || !update_pos_flag || reset_spindle_position_required()) return;
    rt_data_reset();
    if (nc_reset->reset_trigger == HMI_RESET) {
        tapreturn_update(NULL);
        tapreturn_refresh_return_param();
    }
    rerun_data_reset_keep_breakpoint();
    s_phase = RESET_COMPLETE;
    nc_reset->is_resetted = 1;
    /* 复位完成即一次新的显式恢复：解除故障期间因残留 Fault 反馈粘连的
     * ENABLE_FAILED，让下一拍按当前需求重新准备模式/使能。否则故障复位后
     * X/Y/Z 会一直停在 ENABLE_FAILED，表现为"复位后轴不动、参数正常"。
     * 主轴力矩需求在这里一并清零，仍保持下使能，等新指令再使能。 */
    alarm_reset_latch_clear();
}
