#include "rt_drive_control.h"
/** 只负责 M29 的准备：可选的定向/停车、切 CSP、坐标对齐、交接。
 * 位置下发与长期保持归运动模块。 */
#include "spindle_internal.h"
#include "emcmot.h"
#include "motion_csp_stop.h"
#include "rt_hal_adapter.h"
#include "alarm.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>

#define M29_ZERO_RPM 1.0
#define M29_POSITION_TOL 10.0
#define M29_STABLE_CYCLES 10U
#define M29_PREP_CYCLES 10000U
#define M29_ALARM_TIMEOUT 2100
#define M29_ALARM_FIN 2101
#define M29_ALARM_REJECT 2102

static bool read_feedback(HalAxisSnapshot* a)
{
    return spindle_hal_snapshot(a)==HAL_OK &&
        (a->feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_POSITION|HAL_FEEDBACK_SPEED))==
        (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_POSITION|HAL_FEEDBACK_SPEED) &&
        isfinite(a->actual_pos) && isfinite(a->actual_speed_rpm);
}
static void prepare_failed(const char* reason)
{
    g_spindle.m29_phase=SPINDLE_M29_ERROR;
    g_spindle.m29_active=g_spindle.m29_feedback_stable=false;
    g_spindle.reset_requires_start=true;
    g_spindle.start_request_pending=g_spindle.deferred_start_pending=false;
    g_spindle.command_state=g_spindle.run_state=SPINDLE_STOP;
    g_spindle.command_speed=0;
    g_spindle.velocity_planner.enable=false;
    g_spindle.velocity_planner.target_vel=g_spindle_hal.target_velocity=0;
    spindle_internal_clear_velocity_arrival();
    spindle_hal_mode_switch_reset();
    motion_spindle_release_position();
    (void)alarm_spindle_request_torque(false);
    setAlarm(ALARM_AX,ALARM_ERROR,M29_ALARM_TIMEOUT);
    LOG(LOG_LEVEL_ERR,"M29准备失败：%s",reason);
}
static void begin_preparation(spindle_m29_phase_t phase)
{
    motion_spindle_release_position();
    spindle_internal_clear_velocity_arrival();
    g_spindle.m29_active=true;g_spindle.m29_phase=phase;
    g_spindle.m29_feedback_stable=false;
    g_spindle.m29_stop_stable_cycles=0;g_spindle.orient_elapsed_cycles=0;
    g_spindle.reset_requires_start=false;g_spindle.m19_active=false;
    g_spindle.orient_hold_stop=true;g_spindle.csv_switch_pending=false;
    g_spindle.start_request_pending=g_spindle.deferred_start_pending=false;
    g_spindle.command_state=g_spindle.run_state=SPINDLE_STOP;
    g_spindle.run_authorized=g_spindle.program_run_authorized=false;
    g_spindle.command_speed=0;
}
static bool start_alignment(void)
{
    HalAxisSnapshot a;
    if(!read_feedback(&a))return false;
    g_spindle.m29_expected_offset=a.coordinate_offset-a.actual_pos;
    g_spindle.m29_hold_submitted=false;
    g_spindle.m29_stop_stable_cycles=0;g_spindle.orient_elapsed_cycles=0;
    return isfinite(g_spindle.m29_expected_offset) && rt_hal_axis_set_pos(RT_HAL_AXIS_SP,0)==HAL_OK;
}
static void update_alignment(void)
{
    HalAxisSnapshot a;
    const int awaited=g_spindle.m29_hold_submitted?HAL_ISSUE_POSITION:HAL_ISSUE_SET_POSITION;
    if(!read_feedback(&a) || a.drive_state!=HAL_DRIVE_OPERATION_ENABLED ||
       a.actual_mode!=HAL_AXIS_MODE_CSP ||
       (a.request_rejection.category==awaited && a.request_rejection.code)) {
        prepare_failed("坐标对齐反馈或请求异常");return;
    }
    if(fabs(a.coordinate_offset-g_spindle.m29_expected_offset)>1e-6)return;
    if(!g_spindle.m29_hold_submitted) {
        if(rt_hal_request_axis_pos(RT_HAL_AXIS_SP,0)!=HAL_OK)prepare_failed("保持目标被拒绝");
        else g_spindle.m29_hold_submitted=true;
        return;
    }
    if(!(a.command_valid&HAL_COMMAND_POSITION) || fabs(a.command_pos)>1e-6 ||
       fabs(a.actual_speed_rpm)>M29_ZERO_RPM || fabs(a.command_pos-a.actual_pos)>M29_POSITION_TOL) {
        g_spindle.m29_stop_stable_cycles=0;return;
    }
    if(++g_spindle.m29_stop_stable_cycles<M29_STABLE_CYCLES)return;
    if(!motion_spindle_take_position(0)) { prepare_failed("运动模块接管失败");return; }
    g_spindle_hal.target_position=0;
    g_spindle.m29_phase=SPINDLE_M29_READY;g_spindle.m29_feedback_stable=true;
}
/* 两条准备路径共用的状态推进：模式就绪 → 显式使能 → 坐标对齐。 */
static void update_csp_preparation(spindle_m29_phase_t alignment_phase)
{
    HalAxisSnapshot a;
    if(spindle_hal_mode_switch_result()!=HAL_OK) { prepare_failed("CSP模式准备失败");return; }
    if(spindle_hal_snapshot(&a)!=HAL_OK || !(a.feedback_valid&HAL_FEEDBACK_STATUS)) {
        prepare_failed("CSP状态失效");return;
    }
    /* D32：模式准备 READY 与实际模式反馈是两件事。台达要等独立的使能完成
     * 之后才可能报出实际模式。 */
    if(!spindle_hal_mode_switch_is_done() || a.drive_state!=HAL_DRIVE_OPERATION_ENABLED)return;
    if(!read_feedback(&a) || a.actual_mode!=HAL_AXIS_MODE_CSP || fabs(a.actual_speed_rpm)>M29_ZERO_RPM)return;
    if(!start_alignment()) { prepare_failed("软件起点设置被拒绝");return; }
    g_spindle.m29_phase=alignment_phase;
}

bool spindle_m29_is_active(void) { return g_spindle.m29_active; }
bool spindle_m29_has_error(void) { return g_spindle.m29_phase==SPINDLE_M29_ERROR; }
void spindle_report_m29_prepare_rejected(void) { setAlarm(ALARM_AX,ALARM_ERROR,M29_ALARM_REJECT); }
void spindle_report_m29_handshake_timeout(void) { setAlarm(ALARM_AX,ALARM_ERROR,M29_ALARM_FIN); }

bool spindle_request_m29(void)
{
    /* 复位期间拒绝新的业务请求，不能顺手把状态打成 ERROR——那会中断已经在跑的
     * 受控停车。这里只给调用方返回 false。 */
    if(!reset_spindle_business_allowed() || rt_data->sys_state[SYS_ESTOP_INPUT] ||
       rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])return false;
    if(spindle_hal_mode()==HAL_AXIS_MODE_CSP) {
        const int ready=motion_spindle_prepare_velocity();
        if(ready<0) {
            HalAxisSnapshot a={0};
            const int rc=spindle_hal_snapshot(&a);
            LOG(LOG_LEVEL_ERR,"M29请求被拒：位置控制交接条件不可用 rc=%d valid=0x%x command_valid=0x%x drive=%d mode=%d rpm=%.3f pos=%.6f target=%.6f",
                rc,(unsigned)a.feedback_valid,(unsigned)a.command_valid,(int)a.drive_state,
                (int)a.actual_mode,a.actual_speed_rpm,a.actual_pos,a.command_pos);
            return false; /* caller retains the existing rejection alarm/cleanup */
        }
        if(ready==0) {
            LOG(LOG_LEVEL_INFO,"M29请求被拒：位置运动尚未结束或尚未停稳，不排队重试");
            return false;
        }
    }
    begin_preparation(SPINDLE_M29_ORIENTING);
    g_spindle.pending_event=SPINDLE_EVENT_ORIENT;
    return true;
}

bool spindle_request_m29_no_orient(void)
{
    HalAxisSnapshot a;
    int32_t orient;
    const bool switching=spindle_hal_mode_switch_is_pending();
    if(spindle_internal_stop_is_required(&g_spindle) ||
       rt_data->sys_state[SYS_ESTOP_INPUT] || rt_data->axis_data[AXIS_SP].status.is_disable ||
       rt_data->axis_data[AXIS_SP].status.is_error || g_spindle.m29_active ||
       g_spindle.orient_phase!=SPINDLE_ORIENT_IDLE || g_spindle.orient_request_pending ||
       rt_hal_spindle_orient_state(RT_HAL_AXIS_SP,&orient)!=HAL_OK || orient!=RT_HAL_ORIENT_NONE ||
       spindle_hal_snapshot(&a)!=HAL_OK || !(a.feedback_valid&HAL_FEEDBACK_STATUS) ||
       (a.raw_status&0x0008U) ||
       ((a.feedback_valid&HAL_FEEDBACK_ERROR_CODE) && a.drive_error_code) ||
       !isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) ||
       rt_data->axis_data[AXIS_SP].max_feed_acc<=0)return false;
    if((a.feedback_valid&(HAL_FEEDBACK_POSITION|HAL_FEEDBACK_SPEED))!=
       (HAL_FEEDBACK_POSITION|HAL_FEEDBACK_SPEED) || !isfinite(a.actual_pos) || !isfinite(a.actual_speed_rpm))return false;
    const bool mode_known=(a.feedback_valid&HAL_FEEDBACK_MODE)!=0;
    if((!mode_known || (a.actual_mode!=HAL_AXIS_MODE_CSV && a.actual_mode!=HAL_AXIS_MODE_CSP)) &&
       fabs(a.actual_speed_rpm)>M29_ZERO_RPM)return false;
    if(mode_known && a.actual_mode==HAL_AXIS_MODE_CSP && a.drive_state==HAL_DRIVE_OPERATION_ENABLED) {
        if(fabs(a.actual_speed_rpm)>M29_ZERO_RPM ||
           (!g_spindle.reset_requires_start &&
            (!(a.command_valid&HAL_COMMAND_POSITION) || fabs(a.command_pos-a.actual_pos)>M29_POSITION_TOL)))return false;
        if(!switching && !g_spindle.csv_switch_pending &&
           rt_hal_request_axis_pos(RT_HAL_AXIS_SP,a.actual_pos)!=HAL_OK)return false;
    }
    const bool wait_csv=switching || g_spindle.csv_switch_pending || !mode_known ||
        (a.actual_mode!=HAL_AXIS_MODE_CSV && a.actual_mode!=HAL_AXIS_MODE_CSP);
    if(wait_csv && spindle_hal_request_mode(HAL_AXIS_MODE_CSV)!=HAL_OK)return false;
    begin_preparation(wait_csv?SPINDLE_M29_NO_ORIENT_WAIT_CSV:SPINDLE_M29_NO_ORIENT_STOPPING);
    g_spindle.pending_event=SPINDLE_EVENT_NONE;
    g_spindle.velocity_planner.current_vel=a.actual_speed_rpm;
    g_spindle.velocity_planner.target_vel=0;g_spindle.velocity_planner.enable=false;
    g_spindle_hal.target_position=a.actual_pos;
    if(!wait_csv)spindle_hal_mode_switch_reset();
    return true;
}

bool spindle_internal_m29_no_orient_preparing(void)
{
    return g_spindle.m29_phase>=SPINDLE_M29_NO_ORIENT_WAIT_CSV &&
           g_spindle.m29_phase<=SPINDLE_M29_NO_ORIENT_ALIGNING;
}
void spindle_internal_update_m29_no_orient(void)
{
    HalAxisSnapshot a;
    int32_t orient;
    if(spindle_internal_stop_is_required(&g_spindle) ||
       rt_data->axis_data[AXIS_SP].status.is_disable || rt_data->axis_data[AXIS_SP].status.is_error ||
       spindle_hal_snapshot(&a)!=HAL_OK || !(a.feedback_valid&HAL_FEEDBACK_STATUS) ||
       (a.raw_status&0x0008U) ||
       ((a.feedback_valid&HAL_FEEDBACK_ERROR_CODE) && a.drive_error_code) ||
       !isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) || rt_data->axis_data[AXIS_SP].max_feed_acc<=0 ||
       rt_hal_spindle_orient_state(RT_HAL_AXIS_SP,&orient)!=HAL_OK || orient!=RT_HAL_ORIENT_NONE ||
       g_spindle.pending_event==SPINDLE_EVENT_ORIENT || g_spindle.pending_event==SPINDLE_EVENT_RELEASE_ORIENT ||
       ++g_spindle.orient_elapsed_cycles>=M29_PREP_CYCLES) {
        prepare_failed("无预定向准备中止");return;
    }
    g_spindle.pending_event=SPINDLE_EVENT_NONE;
    (void)alarm_spindle_request_torque(true);
    switch(g_spindle.m29_phase) {
    case SPINDLE_M29_NO_ORIENT_WAIT_CSV:
        if(spindle_hal_mode_switch_result()!=HAL_OK) { prepare_failed("CSV模式准备失败");return; }
        if(!spindle_hal_mode_switch_is_done() || !(a.feedback_valid&HAL_FEEDBACK_MODE) ||
           a.actual_mode!=HAL_AXIS_MODE_CSV || a.drive_state!=HAL_DRIVE_OPERATION_ENABLED)return;
        g_spindle.m29_phase=SPINDLE_M29_NO_ORIENT_STOPPING;
        break;
    case SPINDLE_M29_NO_ORIENT_STOPPING: break;
    case SPINDLE_M29_NO_ORIENT_SWITCHING:
        update_csp_preparation(SPINDLE_M29_NO_ORIENT_ALIGNING);return;
    case SPINDLE_M29_NO_ORIENT_ALIGNING: update_alignment();return;
    default:return;
    }
    if(!read_feedback(&a)) { prepare_failed("停车反馈失效");return; }
    const int stopped=a.actual_mode==HAL_AXIS_MODE_CSV?spindle_prepare_csv_stop_step():
        (fabs(a.actual_speed_rpm)<=M29_ZERO_RPM?1:0);
    if(stopped<0) { prepare_failed("CSV停车执行失败");return; }
    if(stopped)++g_spindle.m29_stop_stable_cycles;else g_spindle.m29_stop_stable_cycles=0;
    if(g_spindle.m29_stop_stable_cycles<M29_STABLE_CYCLES)return;
    if(spindle_hal_request_mode(HAL_AXIS_MODE_CSP)!=HAL_OK) { prepare_failed("CSP模式请求被拒绝");return; }
    g_spindle.m29_phase=SPINDLE_M29_NO_ORIENT_SWITCHING;g_spindle.orient_elapsed_cycles=0;
}
void spindle_internal_update_m29(void)
{
    if(!g_spindle.m29_active || spindle_internal_m29_no_orient_preparing() ||
       g_spindle.m29_phase==SPINDLE_M29_READY)return;
    switch(g_spindle.m29_phase) {
    case SPINDLE_M29_ORIENTING:
        if(g_spindle.orient_phase==SPINDLE_ORIENT_DONE) {
            /* 真正的 CSV→CSP 模式切换会让驱动器取消定向。H1 不支持同模式下
             * 假装正常释放的做法。 */
            if(spindle_hal_mode()==HAL_AXIS_MODE_CSP || spindle_hal_request_mode(HAL_AXIS_MODE_CSP)!=HAL_OK) {
                prepare_failed("定向到CSP交接被拒绝");return;
            }
            g_spindle.m29_phase=SPINDLE_M29_SWITCHING_CSP;
            g_spindle.orient_phase=SPINDLE_ORIENT_IDLE;
            g_spindle.orient_request_pending=g_spindle.orient_submitted=false;
            g_spindle.orient_exit_submitted=false;
            g_spindle.orient_elapsed_cycles=0;
            (void)alarm_spindle_request_torque(true);
        } else if(g_spindle.orient_phase==SPINDLE_ORIENT_IDLE) {
            if(g_spindle.reset_requires_start) { prepare_failed("定向失败");return; }
            g_spindle.orient_request_pending=true;g_spindle.orient_phase=SPINDLE_ORIENT_WAIT_ZERO;
            g_spindle.orient_elapsed_cycles=0;
            (void)alarm_spindle_request_torque(true);
        }
        return;
    case SPINDLE_M29_SWITCHING_CSP:
    case SPINDLE_M29_ALIGNING:
        if(++g_spindle.orient_elapsed_cycles>=M29_PREP_CYCLES) { prepare_failed("CSP准备超时");return; }
        if(g_spindle.m29_phase==SPINDLE_M29_SWITCHING_CSP)update_csp_preparation(SPINDLE_M29_ALIGNING);
        else update_alignment();
        return;
    default:prepare_failed("无效准备阶段");return;
    }
}

bool spindle_m29_is_ready(void)
{
    HalAxisSnapshot a;
    return !spindle_internal_stop_is_required(&g_spindle) && g_spindle.m29_active &&
        !g_spindle.reset_requires_start && g_spindle.m29_phase==SPINDLE_M29_READY &&
        g_spindle.m29_feedback_stable && motion_spindle_position_active() && read_feedback(&a) &&
        a.actual_mode==HAL_AXIS_MODE_CSP && a.drive_state==HAL_DRIVE_OPERATION_ENABLED;
}
void spindle_complete_m29(void)
{
    if(!spindle_m29_is_ready())return;
    g_spindle.m29_active=false;g_spindle.m29_phase=SPINDLE_M29_IDLE;
    g_spindle.m29_feedback_stable=false;g_spindle.orient_hold_stop=false;
    g_spindle.command_state=SPINDLE_STOP;g_spindle.command_speed=0;
    g_spindle.run_authorized=g_spindle.program_run_authorized=false;
    spindle_internal_clear_velocity_arrival();
}
void spindle_internal_publish_rigid_state(void)
{
    rt_data->sys_state[SYS_IS_RIGID]=motion_spindle_position_active() || motion_csp_stop_pending() ||
        (g_spindle.m29_phase!=SPINDLE_M29_IDLE && g_spindle.m29_phase!=SPINDLE_M29_ERROR);
}
