#include "rt_drive_control.h"
#include "alarm.h"
/**
 * @file spindle_speed.c
 * @brief 主轴 CSV 速度规划、到速反馈与停止完成判定。
 *
 * 软件规划器只产生受加速度约束的目标转速。M3/M4 完成必须同时满足
 * 软件目标已到达、驱动器处于 CSV/Operation Enabled 且实际转速位于
 * 允许误差带内；M5 完成则要求规划速度、目标速度和实际速度全部归零。
 */

#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <regapi.h>

#define SPINDLE_CONTROL_PERIOD_SECONDS (0.001)
#define SPINDLE_SPEED_EPSILON          (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON    (1.0)
#define SPINDLE_ACTUAL_SPEED_EPSILON   (1.0)
#define SPINDLE_SPEED_ARRIVAL_TOL      (0.05)
#define SPINDLE_SPEED_ARRIVAL_MIN_TOL  (5.0)
#define SPINDLE_ALARM_MISSING_SPEED    (20)
#define ALARM_RESULT_ALREADY_EXISTS    (-8)

/* 内部函数声明 */

static bool spindle_report_missing_speed_alarm(const char *message);
static double spindle_calculate_target_velocity(const spindle_context_t *context);
static void spindle_update_planner_state(spindle_context_t *context);
static bool spindle_is_velocity_arrived(void);
static bool spindle_velocity_feedback_blocked(void);
static bool spindle_stop_is_complete(void);
static bool spindle_is_rotating(void);

/* 外部函数定义 */

void spindle_set_plc_speed(double speed)
{
    /* PLC 线程直接写入；当前没有与 speed_source 成组发布的同步机制。 */
    g_spindle.plc_speed = speed;
}

void spindle_set_speed_source(spindle_speed_source_t source)
{
    g_spindle.speed_source = source;
}

double spindle_get_command_speed(void)
{
    return g_spindle.speed_source == SPINDLE_SPEED_SOURCE_PLC ?
        g_spindle.plc_speed : g_spindle.command_speed;
}

double spindle_get_program_command_speed(void)
{
    return g_spindle.remembered_program_speed;
}

void spindle_internal_fail_velocity(int result)
{
    /* 现场只看到"HAL 错误 rc=262"时无法判断是哪条速度路径失败。
     * 这里在置错前打一条完整上下文，便于区分：驱动器掉使能、停止约束、
     * 模式不符、定向/复位占用，还是请求本身被拒。 */
    HalAxisSnapshot axis;
    const int snap_rc=spindle_hal_snapshot(&axis);
    LOG(LOG_LEVEL_ERR,
        "主轴速度业务失败：rc=%d snap=%d ctl=%d mode=%d/%d drive=%d block=0x%x rpm=%.1f planner=%.1f/%.1f run=%d orient=%d pending=%d m29=%d reset_lock=%d rej=%d/%d exec=%d/%d",
        result,snap_rc,snap_rc==HAL_OK?(int)axis.control_state:-1,
        snap_rc==HAL_OK?(int)axis.actual_mode:-1,snap_rc==HAL_OK?(int)axis.target_mode:-1,
        snap_rc==HAL_OK?(int)axis.drive_state:-1,
        snap_rc==HAL_OK?(unsigned)axis.motion_block_reasons:0u,
        snap_rc==HAL_OK?axis.actual_speed_rpm:NAN,
        (double)g_spindle.velocity_planner.current_vel,
        (double)g_spindle.velocity_planner.target_vel,
        (int)g_spindle.run_state,(int)g_spindle.orient_phase,(int)g_spindle.pending_event,
        (int)g_spindle.m29_active,(int)g_spindle.reset_requires_start,
        snap_rc==HAL_OK?(int)axis.request_rejection.category:-1,
        snap_rc==HAL_OK?(int)axis.request_rejection.code:-1,
        snap_rc==HAL_OK?(int)axis.execution_error.category:-1,
        snap_rc==HAL_OK?(int)axis.execution_error.code:-1);
    g_spindle_hal.velocity_submitted=false;
    g_spindle.run_authorized=g_spindle.program_run_authorized=false;
    g_spindle.manual_run_uses_program_speed=false;
    g_spindle.start_request_pending=g_spindle.deferred_start_pending=false;
    g_spindle.csv_switch_pending=false;
    g_spindle.reset_requires_start=true;
    g_spindle.run_state=SPINDLE_STOP;
    g_spindle.velocity_planner.enable=false;
    g_spindle.velocity_planner.target_vel=0;
    spindle_internal_clear_velocity_arrival();
    (void)alarm_spindle_request_torque(false);
    alarm_report_hal_error(result);
}

bool spindle_internal_update_velocity(void)
{
    /*
     * 仅在驱动器处于 CSV 且允许速度控制时调用。规划器先计算 current_vel，
     * 再写入 HAL 缓存；EtherCAT 主循环负责把缓存发送到 60FF。
     */
    spindle_velocity_planner_t *planner = &g_spindle.velocity_planner;

    HalAxisSnapshot axis;
    int error=spindle_hal_snapshot(&axis);
    /* 只消费本条速度请求的回执，不读取其他业务遗留的拒绝槽。 */
    if(error==HAL_OK && g_spindle_hal.velocity_submitted) {
        int32_t result;
        const int rc=rt_hal_spindle_speed_result(RT_HAL_AXIS_SP,g_spindle_hal.velocity_request_id,&result);
        if(rc==HAL_ERROR_BUSY)return false;
        error=rc==HAL_OK?result:rc;
        g_spindle_hal.velocity_submitted=false;
        if(error==HAL_OK && axis.execution_error.category==HAL_ISSUE_SPINDLE_SPEED)
            error=axis.execution_error.code;
    }
    if(error==HAL_OK && spindle_is_rotating() &&
       !isfinite(spindle_internal_get_selected_speed(&g_spindle)))error=HAL_ERROR_ARGUMENT;
    if(error!=HAL_OK) {
        /* 停止约束/下使能已经在 HAL 侧生效时，速度目标本来就不再受理；
         * 这是停车收口的正常结果，不是速度业务故障。清提交标记后交给
         * 停止或复位路径，不要报警，也不要锁成"等待新启动"。 */
        if(error==HAL_ERROR_NOT_ENABLED &&
           (axis.motion_block_reasons & (HAL_BLOCK_STOP_REQUIRED|HAL_BLOCK_NOT_ENABLED))) {
            g_spindle_hal.velocity_submitted=false;
            return false;
        }
        spindle_internal_fail_velocity(error);
        return false;
    }

    planner->max_acc = rt_data->axis_data[AXIS_SP].max_feed_acc;
    /* enable=false 时，规划器按最大减速度将 current_vel 拉回零。 */
    planner->enable = spindle_is_rotating() && !spindle_hal_mode_switch_is_pending();
    planner->target_vel = 0.0;

    if (planner->enable)
    {
        planner->target_vel = spindle_calculate_target_velocity(&g_spindle);
    }

    spindle_update_planner_state(&g_spindle);

    /*
     * 规划器输出是本周期的目标速度。这里只更新 HAL 缓存，真正写入
     * EtherCAT 60FF 由主站发送周期统一完成。
     */
    g_spindle_hal.target_velocity = planner->current_vel;
    /* 定向类动作（定向/保持/退出/交接）在途时，写速度必被 HAL 以 BUSY 拒；
     * 这是过渡态而非业务故障（现场 M19/M03/G4 循环中按复位就撞过）。本拍不写，
     * 等定向动作收口后再写；其他动作（使能/模式）保持原逻辑，不遮蔽拒绝。 */
    if(axis.control_state==HAL_CONTROL_ORIENTING||
       axis.control_state==HAL_CONTROL_ORIENT_HOLD||
       axis.control_state==HAL_CONTROL_EXITING_ORIENT||
       axis.control_state==HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN)return true;
    const int rc=rt_hal_request_spindle_speed_tracked(RT_HAL_AXIS_SP,
                                       fabs((double)g_spindle_hal.target_velocity),
                                       g_spindle_hal.target_velocity > 0 ? 1 :
                                       (g_spindle_hal.target_velocity < 0 ? -1 : 0),
                                       &g_spindle_hal.velocity_request_id);
    if(rc!=HAL_OK) {
        spindle_internal_fail_velocity(rc);
        return false;
    }
    g_spindle_hal.velocity_submitted=true;
    spindle_internal_finish_stop_if_ready();
    return true;
}

bool spindle_internal_validate_start_request(const spindle_context_t *context)
{
    /* 防止没有有效 S 值时误启动主轴；报警只在首次出现时记录。 */
    const double selected_speed = spindle_internal_get_selected_speed(context);
    const bool selected_speed_is_finite = isfinite(selected_speed);

    if (!selected_speed_is_finite)
    {
        return spindle_report_missing_speed_alarm("主轴速度值缺省或无效");
    }
    if (!isfinite(context->remembered_program_speed))
    {
        return spindle_report_missing_speed_alarm(
            "主轴速度值缺省，请先在MDI中执行M3/M4指令。");
    }

    return true;
}

double spindle_internal_get_selected_speed(const spindle_context_t *context)
{
    if (context->manual_run_uses_program_speed)
    {
        return context->remembered_program_speed;
    }

    /* G33.7 选择 PLC 速度，否则使用 CNC M3/M4 的 S 值。 */
    if (context->speed_source == SPINDLE_SPEED_SOURCE_PLC)
    {
        return context->plc_speed;
    }

    return context->command_speed;
}

void spindle_internal_sync_planner_to_actual(void)
{
    g_spindle.velocity_planner.current_vel = spindle_hal_actual_velocity();
    g_spindle.velocity_planner.target_vel = 0.0;
    g_spindle.velocity_planner.enable = false;
    g_spindle.velocity_planner.active =
        fabs(g_spindle.velocity_planner.current_vel) > SPINDLE_SPEED_EPSILON;
    spindle_internal_clear_velocity_arrival();
}

void spindle_internal_finish_stop_if_ready(void)
{
    /* 只有规划器速度和驱动器实际速度都为零，才允许完成停止握手。 */
    /* 停止完成要求软件规划速度与驱动器实际速度都为零（仅清 60FF 不代表已停，尤其在下使能
     * 或反馈延迟时）；待启动、定向窗口、模式切换、M29 期间一律不算。 */
    if (!spindle_stop_is_complete())
    {
        return;
    }

    if (spindle_hal_mode() >= 0)
    {
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        (void)alarm_spindle_request_torque(false);
    }

    /* 置 ARRIVED 并锁存完成反馈：F45.3 由 PLC 发布层只读查询，保持到
     * 新指令调用 spindle_internal_clear_velocity_arrival() 清除为止。 */
    g_spindle.velocity_state = SPINDLE_VELOCITY_ARRIVED;
    spindle_internal_report_mcode_complete();
}

void spindle_internal_clear_velocity_arrival(void)
{
    g_spindle.mcode_complete = false;
    g_spindle.velocity_state = SPINDLE_VELOCITY_RAMPING;
    clearRegBit(REG_F, 45, 3);
}

bool spindle_velocity_is_ready(void)
{
    /* 控制权还没交接（待启动/定向/模式切换/M29/命令未执行）一律不算到速。 */
    if (spindle_velocity_feedback_blocked())
    {
        return false;
    }
    if (!g_spindle.mcode_complete)
    {
        return false;
    }
    if (g_spindle.velocity_state != SPINDLE_VELOCITY_ARRIVED)
    {
        return false;
    }
    /* 停止：ARRIVED 由 spindle_internal_finish_stop_if_ready 在确认零速后置位，这里再按停止
     * 完成判据复核一次，不只凭 run_state == STOP 放行。 */
    if (g_spindle.run_state == SPINDLE_STOP)
    {
        return spindle_stop_is_complete();
    }
    return spindle_is_velocity_arrived();
}

void spindle_internal_publish_status(void)
{
    /*
     * 调试构建显示规划器速度（便于离线看斜坡）；实时构建发布驱动器实际
     * 反馈转速，供 HMI 显示和 PLC 的 SPGET 指令使用。
     * VAR_CHAN_ACT_SPDL_SPEED 是 fBit64，必须按 double 写：写 int 会让
     * PLC 的 SPGET 判 type != DOUBLE_TYPE 而拒绝，实际速度永远读不到。
     */
#ifdef DEBUG_MODE_ENABLED
    setVarChanInt(0, VAR_CHAN_ACT_SPDL_SPEED,
                  (int32_t)g_spindle.velocity_planner.current_vel, 0);
#else
    const double actual_rpm = spindle_hal_actual_velocity();

    /* 读反馈失败时保留上一次的值：把 NaN 写进 PLC/HMI 的浮点变量会污染
     * 后续比较（SPGET 会把 NaN 当有效速度）。 */
    if (isfinite(actual_rpm))
    {
        (void)setVarChanDouble(0, VAR_CHAN_ACT_SPDL_SPEED, actual_rpm, 0);
    }
#endif
}

void spindle_internal_report_mcode_complete(void)
{
    /* FIN/完成反馈只允许从未完成变为完成一次，避免重复触发上层握手。 */
    if (!g_spindle.mcode_complete)
    {
        g_spindle.mcode_complete = true;
    }
}

/* 内部函数定义 */

static bool spindle_report_missing_speed_alarm(const char *message)
{
    if (setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_MISSING_SPEED) ==
        ALARM_RESULT_ALREADY_EXISTS)
    {
        return false;
    }

    LOG(LOG_LEVEL_ERR, "%s", message);
    return false;
}

static double spindle_calculate_target_velocity(const spindle_context_t *context)
{
    /* PLC 给定直接作为目标速度；程序或面板记忆转速应用主轴倍率。 */
    const double max_speed = fabs(rt_data->axis_data[AXIS_SP].spdl_max_speed);
    const double override = var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100.0;
    double target_speed = fabs(spindle_internal_get_selected_speed(context));

    /* 规划与到速均使用限幅后的目标，避免超上限 S 指令永久等不到 FIN。 */
    if (context->speed_source != SPINDLE_SPEED_SOURCE_PLC ||
        context->manual_run_uses_program_speed)
    {
        target_speed *= fmax(0.0, override);
    }

    target_speed = fmin(max_speed, target_speed);
    if (context->run_state == SPINDLE_CCW)
    {
        target_speed = -target_speed;
    }

    return target_speed;
}

static void spindle_update_planner_state(spindle_context_t *context)
{
    /* 软件规划器到位后仍需结合 606C 实际反馈，避免提前产生 FIN。 */
    const bool arrived = spindle_velocity_planner_update(&context->velocity_planner, SPINDLE_CONTROL_PERIOD_SECONDS);

    if (!context->velocity_planner.active)
    {
        context->velocity_state = SPINDLE_VELOCITY_ZERO;
        if (context->run_authorized &&
            context->velocity_planner.target_vel == 0.0 &&
            spindle_is_velocity_arrived())
        {
            context->velocity_state = SPINDLE_VELOCITY_ARRIVED;
            spindle_internal_report_mcode_complete();
        }
        return;
    }
    if (arrived)
    {
        /* 软件规划器到达目标不等于电机已经到速，必须继续检查 PDO 反馈。 */
        if (spindle_is_velocity_arrived())
        {
            context->velocity_state = SPINDLE_VELOCITY_ARRIVED;
            spindle_internal_report_mcode_complete();
        }
        else
        {
            context->velocity_state = SPINDLE_VELOCITY_RAMPING;
        }
        return;
    }

    context->velocity_state = SPINDLE_VELOCITY_RAMPING;
}

static bool spindle_is_velocity_arrived(void)
{
    RtHalAxisStatus axis;
    const int status_result = rt_hal_axis_status(RT_HAL_AXIS_SP, &axis);

    /* 到速必须满足 CSV、Operation Enabled、有效反馈和目标误差带条件。 */
    if (spindle_velocity_feedback_blocked() || !spindle_is_rotating())
    {
        return false;
    }
    const double target = g_spindle.velocity_planner.current_vel;
    const double actual = status_result == HAL_OK ? axis.actual_speed_rpm : NAN;
    const double tolerance = fmax(SPINDLE_SPEED_ARRIVAL_MIN_TOL,
                                  fabs(target) * SPINDLE_SPEED_ARRIVAL_TOL);

    /*
     * 零目标要求有效反馈确认为零；非零目标还要求实际转速非零并处于
     * 误差带内。状态字 0x006F == 0x0027（Operation Enabled）与 enabled
     * 双查，避免 HAL 使能镜像滞后时用旧反馈误判到速。
     */
    if (!isfinite(target) || status_result != HAL_OK || !axis.speed_valid ||
        !isfinite(actual))
    {
        return false;
    }
    if (spindle_hal_mode() != HAL_AXIS_MODE_CSV || !spindle_hal_is_enabled() ||
        (spindle_hal_status_word() & 0x006FU) != 0x0027U)
    {
        return false;
    }

    if (fabs(target) <= SPINDLE_SPEED_EPSILON)
    {
        return fabs(actual) <= SPINDLE_ACTUAL_ZERO_EPSILON;
    }
    if (fabs(actual) <= SPINDLE_ACTUAL_SPEED_EPSILON)
    {
        return false;
    }

    return fabs(actual - target) <= tolerance;
}

static bool spindle_velocity_feedback_blocked(void)
{
    if (spindle_internal_stop_is_required(&g_spindle) || g_spindle.reset_requires_start)
    {
        return true;
    }
    /* 新请求尚未接管时，上一条指令的到速/零速都不能算作新指令的完成。 */
    if (g_spindle.start_request_pending || g_spindle.deferred_start_pending)
    {
        return true;
    }
    if (g_spindle.pending_event != SPINDLE_EVENT_NONE)
    {
        return true;
    }
    if (g_spindle.orient_request_pending)
    {
        return true;
    }
    if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.orient_hold_stop)
    {
        return true;
    }
    if (spindle_hal_orientation_state() != RT_HAL_ORIENT_NONE)
    {
        return true;
    }
    /* 模式切换（含 M29 的 CSP 保持）期间不做完成判定。 */
    if (g_spindle.csv_switch_pending || spindle_hal_mode_switch_is_pending())
    {
        return true;
    }
    if (g_spindle.m29_active || g_spindle.m29_phase != SPINDLE_M29_IDLE)
    {
        return true;
    }
    /* 命令与运行状态还没对上：例如刚收到 M3/M4、还在切模式。 */
    return g_spindle.command_state != g_spindle.run_state;
}

static bool spindle_stop_is_complete(void)
{
    if (spindle_velocity_feedback_blocked())
    {
        return false;
    }
    if (g_spindle.run_state != SPINDLE_STOP)
    {
        return false;
    }
    if (fabs(g_spindle.velocity_planner.current_vel) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    if (fabs(g_spindle.velocity_planner.target_vel) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    return fabs((double)spindle_hal_actual_velocity()) <= SPINDLE_ACTUAL_ZERO_EPSILON;
}

static bool spindle_is_rotating(void)
{
    /* 逻辑方向用于决定规划器是否继续输出目标速度。 */
    switch (g_spindle.run_state)
    {
    case SPINDLE_CW:
    case SPINDLE_CCW:
        return true;
    case SPINDLE_STOP:
    default:
        return false;
    }
}

/* 准备流程的调用方只申请走一步 CSV 减速，不自己改规划器速度、也不直接下发。
 * 返回 1 = 规划与实际都已到零，0 = 仍在运动，-1 = 反馈或请求失败。 */
int spindle_prepare_csv_stop_step(void)
{
    HalAxisSnapshot a;
    if(spindle_hal_snapshot(&a)!=HAL_OK ||
       (a.feedback_valid&(HAL_FEEDBACK_SPEED|HAL_FEEDBACK_MODE))!=
       (HAL_FEEDBACK_SPEED|HAL_FEEDBACK_MODE) || a.actual_mode!=HAL_AXIS_MODE_CSV ||
       !isfinite(a.actual_speed_rpm))return -1;
    g_spindle.velocity_planner.max_acc=rt_data->axis_data[AXIS_SP].max_feed_acc;
    g_spindle.velocity_planner.enable=false;
    g_spindle.velocity_planner.target_vel=0;
    (void)spindle_velocity_planner_update(&g_spindle.velocity_planner,SPINDLE_CONTROL_PERIOD_SECONDS);
    g_spindle_hal.target_velocity=g_spindle.velocity_planner.current_vel;
    const double speed=g_spindle_hal.target_velocity;
    /* 已经停住时不要再补一个多余的转速目标——同一拍里调用方可能正在申请模式准备。 */
    if(fabs(speed)<=SPINDLE_SPEED_EPSILON && fabs(a.actual_speed_rpm)<=SPINDLE_ACTUAL_ZERO_EPSILON)return 1;
    if(rt_hal_request_spindle_speed(RT_HAL_AXIS_SP,fabs(speed),(speed>0)-(speed<0))!=HAL_OK)return -1;
    return 0;
}
