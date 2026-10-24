/** CSP 同步停车与最终目标保持；不持有任何主轴业务状态。 */
#include "motion_csp_stop.h"
#include "global.h"
#include "rt_hal_adapter.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <string.h>

/* 停车判据与预算沿用从 spindle_safety.c 原样搬过来的那一套，数值未改。 */
#define STOP_SPEED_RPM 1.0
#define STOP_SP_POSITION 10.0
#define STOP_FEED_POSITION_FLOOR .01
#define STOP_STABLE_CYCLES 10U
#define STOP_TOTAL_CYCLES 30000U
#define STOP_IDLE_CYCLES 10000U
#define STOP_LOG_CYCLES 500U
#define STOP_ALARM 2103
static struct {
    bool pending, hold_valid;
    int feed_axis;
    unsigned cycles, idle_cycles, stable_cycles;
    double hold_target;
} s_stop;

bool motion_csp_stop_pending(void) { return s_stop.pending; }
void motion_csp_stop_cancel(void) { memset(&s_stop,0,sizeof(s_stop)); }

bool motion_csp_stop_begin(void)
{
    HalAxisSnapshot sp;
    if (s_stop.pending) return true;
    if (!cycle_tp || !cycle_tp->plan_block || cycle_tp->exec_cmd != DO_TAP_CMD ||
        (cycle_tp->plan_block->type != RIGID_TAP && cycle_tp->plan_block->type != TAP_RETURN) ||
        rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&sp) != HAL_OK ||
        (sp.feedback_valid & (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE)) !=
            (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE) ||
        sp.drive_state != HAL_DRIVE_OPERATION_ENABLED || sp.actual_mode != HAL_AXIS_MODE_CSP)
        return false;
    int feed_axis;
    switch(cycle_tp->plan_block->plane) {
    case PLANE_XY: feed_axis=AXIS_Z;break;
    case PLANE_XZ: feed_axis=AXIS_Y;break;
    case PLANE_YZ: feed_axis=AXIS_X;break;
    default: return false;
    }
    motion_csp_stop_cancel();
    s_stop.feed_axis=feed_axis;s_stop.pending=true;
    LOG(LOG_LEVEL_INFO,"HMI复位：运动模块接管CSP同步停车，进给轴=%d",feed_axis);
    return true;
}

static double feed_tolerance(void)
{
    double value=rt_data->axis_data[s_stop.feed_axis].pos_tol;
    return isfinite(value) && value>=STOP_FEED_POSITION_FLOOR ? value : STOP_FEED_POSITION_FLOOR;
}
static void stop_failed(const char* reason)
{
    LOG(LOG_LEVEL_ERR,"CSP复位停车失败：%s cycles=%u idle=%u stable=%u feed=%d",
        reason,s_stop.cycles,s_stop.idle_cycles,s_stop.stable_cycles,s_stop.feed_axis);
    s_stop.pending=false;s_stop.hold_valid=false;
    reset_stop_executor_finished(false);
    setAlarm(ALARM_AX,ALARM_ERROR,STOP_ALARM);
}
static void capture_final_target(void)
{
    if (!s_stop.hold_valid && cycle_tp && cycle_tp->exec_state==EXEC_IDLE) {
        s_stop.hold_target=rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur;
        s_stop.hold_valid=isfinite(s_stop.hold_target);
    }
}
void motion_csp_stop_update(void)
{
    if (!s_stop.pending) return;
    RtHalAxisStatus sp={0},feed={0};
    HalAxisSnapshot snapshot;
    const bool idle=cycle_tp && cycle_tp->exec_state==EXEC_IDLE;
    ++s_stop.cycles;
    if(idle)++s_stop.idle_cycles;else s_stop.idle_cycles=0;
    if(s_stop.cycles>=STOP_TOTAL_CYCLES || (idle && s_stop.idle_cycles>=STOP_IDLE_CYCLES) ||
       rt_hal_axis_status(RT_HAL_AXIS_SP,&sp)!=HAL_OK || !sp.speed_valid || !sp.position_valid || !sp.enabled ||
       rt_hal_axis_status((RtHalAxisSlot)s_stop.feed_axis,&feed)!=HAL_OK || !feed.position_valid ||
       rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&snapshot)!=HAL_OK ||
       !(snapshot.feedback_valid&HAL_FEEDBACK_MODE) || snapshot.actual_mode!=HAL_AXIS_MODE_CSP) {
        stop_failed("超时、反馈失效或CSP控制条件丢失");return;
    }
    capture_final_target();
    /* 有意不判进给速度：保留原有的容漂移位置判据，配合主轴转速/位置与插补空闲。 */
    if(idle && fabs(sp.actual_speed_rpm)<=STOP_SPEED_RPM &&
       fabs(sp.command_pos-sp.actual_pos)<=STOP_SP_POSITION &&
       fabs(feed.command_pos-feed.actual_pos)<=feed_tolerance())++s_stop.stable_cycles;
    else {
        s_stop.stable_cycles=0;
        if(s_stop.cycles%STOP_LOG_CYCLES==0)
            LOG(LOG_LEVEL_DEBUG,"CSP复位停车等待：cycles=%u idle=%u rpm=%.3f sp_err=%.3f feed_err=%.4f tol=%.4f",
                s_stop.cycles,s_stop.idle_cycles,sp.actual_speed_rpm,
                fabs(sp.command_pos-sp.actual_pos),fabs(feed.command_pos-feed.actual_pos),feed_tolerance());
    }
    if(s_stop.stable_cycles>=STOP_STABLE_CYCLES) {
        s_stop.pending=false;
        reset_stop_executor_finished(true);
        LOG(LOG_LEVEL_INFO,"CSP同步停车完成，交公共流程下使能");
    }
}

bool motion_csp_stop_handle_position_output(void)
{
    if(!s_stop.pending)return false;
    capture_final_target();
    /* 插补结束时冻结指令，绝不跟随实际位置；
     * 同一条运动通路同时提供实时轨迹与最终保持目标。 */
    const double target=s_stop.hold_valid?s_stop.hold_target:rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur;
    if(s_stop.hold_valid)rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=target;
    if(!isfinite(target) || rt_hal_request_axis_pos(RT_HAL_AXIS_SP,target)!=HAL_OK)
        stop_failed("位置目标不可用或提交被拒");
    /* 即使失败也要消费掉本拍的输出，绝不能落到普通输出路径。 */
    return true;
}
