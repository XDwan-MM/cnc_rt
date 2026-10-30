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

/* 定向失败统一出口：先落日志再动状态，保证现场能从日志定位是哪一步失败。
 * 快照读取失败时相关字段输出 -1，不能用 0 冒充"有效且空闲"。 */
void spindle_orientation_failed(bool retain_hold, const char *reason) {
    HalAxisSnapshot axis;
    const int rc=spindle_hal_snapshot(&axis);
    LOG(LOG_LEVEL_ERR,
        "主轴定向失败(%s)：retain=%d rc=%d control=%d mode=%d/%d mode_set=%d drive=%d block=0x%x rejection=%d/%d execution=%d/%d",
        reason!=NULL?reason:"未说明",(int)retain_hold,rc,
        rc==HAL_OK?(int)axis.control_state:-1,
        rc==HAL_OK?(int)axis.actual_mode:-1,rc==HAL_OK?(int)axis.target_mode:-1,
        rc==HAL_OK?(int)axis.mode_setting_state:-1,
        rc==HAL_OK?(int)axis.drive_state:-1,
        rc==HAL_OK?(unsigned)axis.motion_block_reasons:0u,
        rc==HAL_OK?(int)axis.request_rejection.category:-1,
        rc==HAL_OK?(int)axis.request_rejection.code:-1,
        rc==HAL_OK?(int)axis.execution_error.category:-1,
        rc==HAL_OK?(int)axis.execution_error.code:-1);
    /* 正常退出被拒绝时保留定向保持，不把拒绝当作已释放。 */
    g_spindle.m19_active=false;
    g_spindle.orient_request_pending=g_spindle.orient_submitted=false;
    g_spindle.orient_exit_submitted=false;
    g_spindle.orient_exit_requested=false;
    g_spindle.deferred_start_pending=g_spindle.start_request_pending=false;
    g_spindle.reset_requires_start=true;g_spindle.orient_hold_stop=true;
    g_spindle.run_authorized=g_spindle.program_run_authorized=false;
    g_spindle.orient_phase=retain_hold?SPINDLE_ORIENT_DONE:SPINDLE_ORIENT_IDLE;
    g_spindle.orient_elapsed_cycles=0;
    g_spindle.orient_stall_cycles=0;
    spindle_internal_clear_velocity_arrival();
    if(!retain_hold)(void)alarm_spindle_request_torque(false);
    setAlarm(ALARM_AX,ALARM_ERROR,SPINDLE_ALARM_ORIENT_TIMEOUT);
}
bool spindle_orientation_submit(void) {
    /* 受理不代表硬件已开始；后续周期还须检查拒绝槽和控制状态。 */
    if(g_spindle.orient_submitted)return true;
    int rc=rt_hal_spindle_request_orient(RT_HAL_AXIS_SP);
    if(rc)return false;
    g_spindle.orient_submitted=true;
    g_spindle.orient_stall_cycles=0;
    return true;
}
bool spindle_orientation_hal_is_done(void) {
    /* 不直接复用 6041.bit10；先低后高及动作延时由 HAL 执行。 */
    HalAxisSnapshot axis;
    return spindle_hal_snapshot(&axis)==HAL_OK &&
        (axis.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE))==
        (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE) &&
        axis.drive_state==HAL_DRIVE_OPERATION_ENABLED &&
        axis.actual_mode==axis.target_mode && axis.control_state==HAL_CONTROL_ORIENT_HOLD;
}
spindle_orient_release_t spindle_orientation_release(void) {
    /* CSV 保持零速，CSP 保持当前反馈位置；均须等后续快照确认退出。
     * 关键：只有真的把退出交给 HAL（或确认已在退出中）才返回 SUBMITTED。
     * 没有在途动作时必须返回 IDLE，由调用方收口——否则会进入无人推进的
     * RELEASING（复位后定向尚未提交、下一条 M3 先到时的现场故障）。
     * 定向仍在 ORIENTING 或 HAL 因"未停稳"返回 BUSY 时是 RETRY：现场第三条
     * M19 紧跟 100 rpm 的 M3 时就会命中，不能当失败报警。 */
    if(g_spindle.orient_exit_submitted)return SPINDLE_ORIENT_RELEASE_SUBMITTED;
    HalAxisSnapshot axis;
    if(spindle_hal_snapshot(&axis)!=HAL_OK)return SPINDLE_ORIENT_RELEASE_FAILED;
    if(axis.control_state==HAL_CONTROL_EXITING_ORIENT ||
       axis.control_state==HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN) {
        /* 已在退出/交接未知：认领这次等待，由后续快照统一收口。 */
        g_spindle.orient_exit_submitted=true;
        return SPINDLE_ORIENT_RELEASE_SUBMITTED;
    }
    if(axis.control_state==HAL_CONTROL_ORIENTING)
        return SPINDLE_ORIENT_RELEASE_RETRY;   /* 等定向到位再退出，避免 BUSY */
    if(axis.control_state!=HAL_CONTROL_ORIENT_HOLD)
        return SPINDLE_ORIENT_RELEASE_IDLE;
    HalOrientExitTarget target={0};
    if(!(axis.feedback_valid&HAL_FEEDBACK_MODE))return SPINDLE_ORIENT_RELEASE_FAILED;
    if(axis.actual_mode==HAL_AXIS_MODE_CSV) {
        target.kind=HAL_ORIENT_EXIT_SPINDLE_SPEED;target.value=0;
    } else if(axis.actual_mode==HAL_AXIS_MODE_CSP &&
              (axis.feedback_valid&HAL_FEEDBACK_POSITION) && isfinite(axis.actual_pos)) {
        target.kind=HAL_ORIENT_EXIT_POSITION;target.value=axis.actual_pos;
    } else return SPINDLE_ORIENT_RELEASE_FAILED;
    {
        const int rc=rt_hal_spindle_exit_orient(RT_HAL_AXIS_SP,&target);
        /* 适配层只是暂存请求，BUSY 通常由下一拍快照的 request_rejection 带出；
         * 这里保留同步返回的判断以兼容直接调用 HAL 的场合。 */
        if(rc==HAL_ERROR_BUSY)return SPINDLE_ORIENT_RELEASE_RETRY;
        if(rc!=HAL_OK)return SPINDLE_ORIENT_RELEASE_FAILED;
    }
    g_spindle.orient_exit_submitted=true;
    g_spindle.orient_stall_cycles=0;
    return SPINDLE_ORIENT_RELEASE_SUBMITTED;
}
/* 退出完成/确认无需退出后的公共收尾；保留 orient_request_pending，
 * 供重复 M19 / M29 在旧保持退出后继续提交新一次定向。 */
static void spindle_orientation_exit_finished(void) {
    g_spindle.orient_submitted=false;
    g_spindle.orient_exit_submitted=false;
    g_spindle.orient_exit_requested=false;
    g_spindle.orient_phase=SPINDLE_ORIENT_IDLE;
    g_spindle.orient_elapsed_cycles=0;
    g_spindle.orient_stall_cycles=0;
    g_spindle.orient_hold_stop=false;
    spindle_internal_sync_planner_to_actual();
    spindle_orientation_apply_deferred_start();
}
void spindle_orientation_complete_release(void) {
    /* 确认没有在途硬件动作后的本地收口：不下发 HAL 退出、不报警，
     * 直接执行定向期间排队的启动；没有排队启动则保持零速使能。 */
    g_spindle.orient_request_pending=false;
    spindle_orientation_exit_finished();
}
void spindle_orientation_abort(void) {
    /* 取消属于停车业务，不等待普通定向退出动作。 */
    g_spindle.orient_request_pending=g_spindle.orient_submitted=false;
    g_spindle.orient_exit_submitted=false;
    g_spindle.orient_exit_requested=false;
    g_spindle.deferred_start_pending=false;
    g_spindle.m19_active=false;
    g_spindle.orient_phase=SPINDLE_ORIENT_IDLE;
    g_spindle.orient_elapsed_cycles=0;
    g_spindle.orient_stall_cycles=0;
    (void)alarm_spindle_request_torque(false);
}
void spindle_begin_m19(void)
{
    /* 此处只登记 M19 握手，实际定向由 PLC G70.6 事件触发。 */
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
    /* 统一走带宽窄仲裁的入口：RELEASE 可以覆盖尚未消费的 ORIENT（表示该定向
     * 从未下发给硬件，由释放流程本地收口），但 CW/CCW 不能覆盖 ORIENT。 */
    spindle_post_event(active ? SPINDLE_EVENT_ORIENT : SPINDLE_EVENT_RELEASE_ORIENT);
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
    /* 已请求退出但硬件还不能退（仍在定向中或未停稳）：按业务预算逐拍重试，
     * 受理后本拍继续走下面的 exit_submitted 等待分支。不能把 BUSY 当失败。 */
    if(g_spindle.orient_exit_requested && !g_spindle.orient_exit_submitted) {
        const spindle_orient_release_t released=spindle_orientation_release();
        if(released==SPINDLE_ORIENT_RELEASE_FAILED) {
            spindle_orientation_failed(true,"释放定向请求被拒绝");return;
        }
        if(released==SPINDLE_ORIENT_RELEASE_IDLE) {
            if(g_spindle.orient_request_pending)spindle_orientation_exit_finished();
            else spindle_orientation_complete_release();
            return;
        }
        if(released==SPINDLE_ORIENT_RELEASE_RETRY) {
            if(++g_spindle.orient_elapsed_cycles>=SPINDLE_ORIENT_TIMEOUT_CYCLES)
                spindle_orientation_failed(true,"等待定向进入可退出状态超时");
            return;
        }
        /* SUBMITTED：本拍刚暂存，真正执行和拒绝都在下一拍快照里；
         * 本拍不要再用旧拒绝记录做判定。 */
        return;
    }
    /* 状态自愈：非 IDLE 阶段既没有待提交请求，也没有在途退出/定向动作时，
     * 说明有人打破了 I1/I2 不变量（例如释放时动作尚未提交）。这里累计到
     * 阈值后本地收口并留日志，避免像现场那样永久停在 RELEASING。 */
    if(g_spindle.orient_phase!=SPINDLE_ORIENT_IDLE &&
       g_spindle.orient_phase!=SPINDLE_ORIENT_DONE &&
       !g_spindle.orient_request_pending && !g_spindle.orient_exit_submitted &&
       !(g_spindle.orient_phase==SPINDLE_ORIENT_RUNNING && g_spindle.orient_submitted)) {
        if(++g_spindle.orient_stall_cycles>=SPINDLE_ORIENT_STALL_CYCLES) {
            LOG(LOG_LEVEL_WARN,"主轴定向阶段%d无在途动作，连续%u拍未推进，本地收口",
                (int)g_spindle.orient_phase,g_spindle.orient_stall_cycles);
            spindle_orientation_complete_release();
            return;
        }
    } else {
        g_spindle.orient_stall_cycles=0;
    }
    HalAxisSnapshot axis;
    if(spindle_hal_snapshot(&axis)!=HAL_OK || !(axis.feedback_valid&HAL_FEEDBACK_STATUS)) {
        spindle_orientation_failed(false,"状态快照失效");return;
    }
    if(g_spindle.orient_exit_submitted) {
        const bool exit_progressing =
            axis.control_state==HAL_CONTROL_EXITING_ORIENT ||
            axis.control_state==HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN;
        const bool exit_rejected =
            (axis.request_rejection.category==HAL_ISSUE_EXIT_ORIENT && axis.request_rejection.code) ||
            (axis.execution_error.category==HAL_ISSUE_EXIT_ORIENT && axis.execution_error.code);
        /* 退出请求由 PDO 线程在下一拍才真正执行，拒绝记录是"每轴最新一条"且会保留：
         * 若退出已经进入 EXITING/HANDOVER，读到的记录必然早于本次请求，不能据此判失败。 */
        if(!exit_progressing && exit_rejected) {
            const bool busy_only =
                axis.request_rejection.category==HAL_ISSUE_EXIT_ORIENT &&
                axis.request_rejection.code==HAL_ERROR_BUSY &&
                !(axis.execution_error.category==HAL_ISSUE_EXIT_ORIENT && axis.execution_error.code);
            if(busy_only) {
                /* HAL 的"未停稳"瞬态：退回待提交状态，保留退出意图，下一拍重试。
                 * 现场 17:55/18:08 两次停机都是这条拒绝，不能升级成 2100。 */
                g_spindle.orient_exit_submitted=false;
                if(++g_spindle.orient_elapsed_cycles>=SPINDLE_ORIENT_TIMEOUT_CYCLES)
                    spindle_orientation_failed(true,"等待定向停稳超时");
                return;
            }
            /* H1 下的 UNSUPPORTED 等明确拒绝，既不是成功释放，也不是自动停车。 */
            spindle_orientation_failed(axis.control_state==HAL_CONTROL_ORIENT_HOLD ||
                               axis.control_state==HAL_CONTROL_ORIENTING,"退出定向请求被拒绝");
            return;
        }
        if(axis.control_state!=HAL_CONTROL_ORIENTING && axis.control_state!=HAL_CONTROL_ORIENT_HOLD &&
           axis.control_state!=HAL_CONTROL_EXITING_ORIENT && axis.control_state!=HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN) {
            /* 退出完成。这里不能清 orient_request_pending：重复 M19 / M29 会先退出
             * 旧保持，再提交新一次定向；待提交请求由下一拍继续推进。 */
            spindle_orientation_exit_finished();
        }
        return;
    }
    if(g_spindle.orient_request_pending) {
        if(g_spindle.orient_submitted) {
            if(axis.request_rejection.category==HAL_ISSUE_ORIENT && axis.request_rejection.code) {
                /* NOT_ENABLED = 轴还在停车/使能收口；清提交标记，下一拍按
                 * 模式与使能条件重发，预算继续累计。其他码仍按失败处理。 */
                if(axis.request_rejection.code==HAL_ERROR_NOT_ENABLED) {
                    g_spindle.orient_submitted=false;
                } else {
                    spindle_orientation_failed(false,"定向请求被拒绝");return;
                }
            }
            if(g_spindle.orient_submitted &&
               (axis.control_state==HAL_CONTROL_ORIENTING||axis.control_state==HAL_CONTROL_ORIENT_HOLD)) {
                g_spindle.orient_request_pending=false;
                g_spindle.orient_phase=SPINDLE_ORIENT_RUNNING;
            } else if(g_spindle.orient_submitted &&
                      axis.execution_error.category==HAL_ISSUE_ORIENT && axis.execution_error.code) {
                spindle_orientation_failed(false,"定向执行失败");return;
            }
        } else {
            /* ENABLE/BUSY 是"上一笔停止还没收口"的瞬态（停车后紧接 M19 必现），
             * 由使能管理重试，这里只等；其他使能拒绝仍是确定性失败。 */
            if(spindle_hal_mode_switch_result()!=HAL_OK ||
               (axis.request_rejection.category==HAL_ISSUE_ENABLE &&
                axis.request_rejection.code &&
                axis.request_rejection.code!=HAL_ERROR_BUSY)) {
                spindle_orientation_failed(false,"定向前的模式/使能准备被拒绝");return;
            }
            if(spindle_hal_mode_switch_is_done()&&spindle_hal_is_enabled()) {
                if(!spindle_orientation_submit())spindle_orientation_failed(false,"定向请求提交失败");
            }
        }
        if(++g_spindle.orient_elapsed_cycles>=SPINDLE_ORIENT_TIMEOUT_CYCLES) {
            spindle_orientation_failed(false,"定向准备超时");return; /* 业务准备总预算，不重开 HAL 动作预算。 */
        }
    }
    if(g_spindle.orient_phase==SPINDLE_ORIENT_RUNNING) {
        if(spindle_orientation_hal_is_done())g_spindle.orient_phase=SPINDLE_ORIENT_DONE;
        else if(axis.control_state!=HAL_CONTROL_ORIENTING) {
            spindle_orientation_failed(false,"定向运行状态丢失");return;
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
    }
    g_spindle.deferred_start_pending = false;

    if (!g_spindle.start_request_pending)
    {
        /* 退出定向不是 M5；无后续指令时保持零速使能，不触发普通停止下使能。 */
        if(!g_spindle.orient_request_pending)g_spindle.orient_hold_stop=true;
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
