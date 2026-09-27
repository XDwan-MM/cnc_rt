#include "spl_control.h"
#include "driver.h"
#include "rt_data.h"
#include "spl_orient.h"
#include "spl_vel_planner.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <parapi.h>

#define SPINDLE_CONTROL_PERIOD_SECONDS (0.001)
#define SPINDLE_SPEED_EPSILON          (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON    (1.0)
#define SPINDLE_ALARM_MISSING_SPEED    (20)
#define ALARM_RESULT_ALREADY_EXISTS    (-8)

spindle_context_t spl_context;

/* ==========================================================================
 * 1. 内部函数声明
 * ========================================================================== */

static bool spindle_stop_is_required(const spindle_context_t *context);
static void spindle_update_command_event(void);
static void spindle_process_event(spindle_context_t *context);
static void spindle_handle_stop(spindle_context_t *context);
static void spindle_handle_rotation(spindle_context_t *context,
                                    spindle_state_t state);
static bool spindle_validate_start_request(const spindle_context_t *context);
static bool spindle_report_missing_speed_alarm(const char *message);
static double spindle_get_selected_speed(const spindle_context_t *context);
static double spindle_calculate_target_velocity(const spindle_context_t *context);
static void spindle_update_driver_control(void);
static void spindle_update_velocity(void);
static void spindle_update_planner_state(spindle_context_t *context);
static void spindle_finish_stop_if_ready(void);
static void spindle_disable_driver(void);
static void spindle_disable_driver_before_operation(void);
static bool spindle_is_rotating(void);
static void spindle_publish_status(void);
static void spindle_report_mcode_complete(void);

/* ==========================================================================
 * 2. 外部函数定义
 * ========================================================================== */

/**
 * @brief 执行一个主轴控制周期。
 *
 * 控制周期按固定顺序完成安全检查、事件处理、定向更新、驱动器控制
 * 和状态发布。主轴控制相关的驱动器细节由本模块统一管理。
 */
void spindle_control_update(void)
{
    if ((rt_data->nc_event == SYS_EVENT_HMI_RESET) ||
        (nc_reset != NULL && nc_reset->is_resetting))
    {
        spl_context.reset_requires_start = true;
    }

    if (spindle_stop_is_required(&spl_context))
    {
        spl_context.spl_event = SPINDLE_EVENT_STOP;
    }
    else
    {
        /* 复位后的旧目标速度不能再次触发启动。 */
        if (spl_context.reset_requires_start &&
            !spl_context.start_request_pending)
        {
            spl_context.spl_event = SPINDLE_EVENT_NONE;
        }
        spindle_update_command_event();
    }

    if (spl_context.reset_requires_start &&
        !spl_context.start_request_pending &&
        (spl_context.spl_event == SPINDLE_EVENT_CW ||
         spl_context.spl_event == SPINDLE_EVENT_CCW))
    {
        /* 丢弃复位前遗留的 PLC/方向边沿，直到新的 M3/M4 到来。 */
        spl_context.spl_event = SPINDLE_EVENT_NONE;
    }

    spindle_process_event(&spl_context);
    spl_orient_update();
    spindle_update_driver_control();
    spindle_publish_status();
}

/* CNC M3/M4/M5 只更新目标数据；目标发生变化时在控制层生成事件。 */
static void spindle_update_command_event(void)
{
    const double selected_speed = spindle_get_selected_speed(&spl_context);
    const double target_speed = isfinite(selected_speed) ?
        fabs(selected_speed) : 0.0;
    const spindle_state_t direction =
        (spl_context.command_state == SPINDLE_CCW) ? SPINDLE_CCW :
        (spl_context.command_state == SPINDLE_CW) ? SPINDLE_CW :
        spl_context.spl_state;
    const double signed_target =
        direction == SPINDLE_CCW ? -target_speed : target_speed;

    if (spl_context.reset_requires_start &&
        !spl_context.start_request_pending)
    {
        return;
    }

    if (fabs(signed_target - spl_context.vel_planner.target_vel) <=
        SPINDLE_SPEED_EPSILON)
    {
        return;
    }

    if (target_speed <= SPINDLE_SPEED_EPSILON)
    {
        spl_context.spl_event = SPINDLE_EVENT_STOP;
    }
    else if (direction == SPINDLE_CCW)
    {
        spl_context.spl_event = SPINDLE_EVENT_CCW;
    }
    else
    {
        spl_context.spl_event = SPINDLE_EVENT_CW;
    }
}

/* ==========================================================================
 * 3. 内部函数定义
 * ========================================================================== */

static void spindle_process_event(spindle_context_t *context)
{
    static bool stop_log_reported = false;

    switch (context->spl_event)
    {
    case SPINDLE_EVENT_STOP:
        spindle_handle_stop(context);
        if (!stop_log_reported)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴收到停止指令，当前速度=%f",
                context->vel_planner.current_vel);
            stop_log_reported = true;
        }
        break;
    case SPINDLE_EVENT_CW:
        spindle_handle_rotation(context, SPINDLE_CW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_CCW:
        spindle_handle_rotation(context, SPINDLE_CCW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_ORIENT:
        spl_orient_request();
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        spl_orient_release();
        break;
    case SPINDLE_EVENT_NONE:
    default:
        break;
    }

    context->spl_event = SPINDLE_EVENT_NONE;
}

static void spindle_update_driver_control(void)
{
    bool velocity_control_ready = false;

    if (Driver_Is_Mode_Switching(&spindle_driver))
    {
        /* Keep the planner decelerating while a driver mode transition is pending. */
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        if (spindle_driver.act_mode == DRV_MODE_CSV)
            spindle_update_velocity();
        else
            spindle_driver.target_vel = 0;
        return;
    }

    if (spindle_driver.act_mode == DRV_MODE_CSV)
    {
        if (spindle_driver.state == DRV_STATE_OPERATION_ENABLED)
        {
            velocity_control_ready = true;
        }
    }

    if (spl_context.spl_state == SPINDLE_STOP)
    {
        if (velocity_control_ready)
        {
            spindle_update_velocity();
        }
        else if (spindle_driver.act_mode == DRV_MODE_NONE)
        {
            spindle_driver.target_vel = 0;
            spindle_finish_stop_if_ready();
        }
        else
        {
            spindle_disable_driver_before_operation();
        }
        return;
    }

    if (velocity_control_ready)
    {
        spindle_update_velocity();
        return;
    }
}

static void spindle_update_velocity(void)
{
    spl_vel_planner_t *planner = &spl_context.vel_planner;

    planner->max_acc = rt_data->axis_data[AXIS_SP].max_feed_acc;
    planner->enable = spindle_is_rotating() && !Driver_Is_Mode_Switching(&spindle_driver);
    planner->target_vel = 0.0;

    if (planner->enable)
    {
        planner->target_vel = spindle_calculate_target_velocity(&spl_context);
    }

    spindle_update_planner_state(&spl_context);

    /* 规划器输出是本周期的目标速度，统一通过 HAL 下发。 */
    spindle_driver.target_vel = (int32_t)planner->current_vel;
    Hal_Driver_Set_Target_Vel(&spindle_driver, spindle_driver.target_vel);

    spindle_finish_stop_if_ready();
}

static bool spindle_stop_is_required(const spindle_context_t *context)
{
    bool stop_required = false;

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
    if (context->spl_event == SPINDLE_EVENT_STOP)
    {
        stop_required = true;
    }

    return stop_required;
}

static void spindle_handle_stop(spindle_context_t *context)
{
    context->spl_state = SPINDLE_STOP;
    context->vel_planner.enable = false;
    context->vel_planner.target_vel = 0.0;

    /* 停止过程保持驱动器使能；已经下使能时不要被重复复位重新使能。 */
    if (spindle_driver.act_mode != DRV_MODE_NONE)
    {
        Driver_Enable(&spindle_driver);
    }
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
}

static void spindle_handle_rotation(spindle_context_t *context,
                                    spindle_state_t state)
{
    const double selected_speed = spindle_get_selected_speed(context);

    /* M3/M4 S0 与 M5 等价。 */
    if (isfinite(selected_speed) &&
        fabs(selected_speed) <= SPINDLE_SPEED_EPSILON)
    {
        spindle_handle_stop(context);
        return;
    }

    if (!spindle_validate_start_request(context))
    {
        return;
    }

    context->spl_state = state;
    if (context->reset_requires_start && !context->start_request_pending)
    {
        return;
    }
    context->reset_requires_start = false;
    context->start_request_pending = false;
    spindle_driver.act_mode = DRV_MODE_CSV;
    Driver_Enable(&spindle_driver);

    LOG(LOG_LEVEL_INFO,
        "主轴收到%s转指令，目标速度=%f",
        state == SPINDLE_CW ? "正" : "反",
        spindle_calculate_target_velocity(context));
}

static bool spindle_validate_start_request(const spindle_context_t *context)
{
    const double selected_speed = spindle_get_selected_speed(context);
    const bool selected_speed_is_finite = isfinite(selected_speed);

    if (!selected_speed_is_finite)
    {
        return spindle_report_missing_speed_alarm("主轴速度值缺省或无效");
    }
    if (!isfinite(context->mcp_target_speed))
    {
        return spindle_report_missing_speed_alarm(
            "主轴速度值缺省，请先在MDI中执行M3/M4指令。");
    }
    if (fabs(context->mcp_target_speed) <= SPINDLE_SPEED_EPSILON)
    {
        return spindle_report_missing_speed_alarm(
            "主轴速度值缺省，请先在MDI中执行M3/M4指令。");
    }

    return true;
}

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

static double spindle_get_selected_speed(const spindle_context_t *context)
{
    if (context->speed_source == SPINDLE_SPEED_SOURCE_PLC)
    {
        return context->plc_speed;
    }

    return context->command_speed;
}

static double spindle_calculate_target_velocity(const spindle_context_t *context)
{
    const double max_speed = fabs(rt_data->axis_data[AXIS_SP].spdl_max_speed);
    const double override = var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100.0;
    double target_speed = fabs(spindle_get_selected_speed(context));

    if (context->speed_source == SPINDLE_SPEED_SOURCE_CNC)
    {
        target_speed *= fmax(0.0, override);
    }

    target_speed = fmin(max_speed, target_speed);
    if (context->spl_state == SPINDLE_CCW)
    {
        target_speed = -target_speed;
    }

    return target_speed;
}

static void spindle_update_planner_state(spindle_context_t *context)
{
    const bool arrived = spl_vel_planner_update(
        &context->vel_planner, SPINDLE_CONTROL_PERIOD_SECONDS);

    if (!context->vel_planner.active)
    {
        context->spl_vel_state = SPINDLE_VEL_ZERO;
        return;
    }
    if (arrived)
    {
        context->spl_vel_state = SPINDLE_VEL_ARRIVED;
        spindle_report_mcode_complete();
        return;
    }

    context->spl_vel_state = SPINDLE_VEL_RAMP;
}

static void spindle_finish_stop_if_ready(void)
{
    if (spl_context.spl_state != SPINDLE_STOP)
    {
        return;
    }
    if (fabs(spl_context.vel_planner.current_vel) > SPINDLE_SPEED_EPSILON)
    {
        return;
    }
    if (fabs((double)spindle_driver.actual_vel) >
        SPINDLE_ACTUAL_ZERO_EPSILON)
    {
        return;
    }

    if (spindle_driver.act_mode == DRV_MODE_NONE)
    {
        if (!spl_context.mcode_feedback)
        {
            /* 停止目标到达零速也必须产生一次到速信号，供 M 代码 FIN 握手。 */
            spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
            spindle_report_mcode_complete();
        }
        return;
    }

    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
    /* 先发布 ARRIVED，让 PLC 在本周期输出 F45.3；下一周期再回到 ZERO。 */
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spindle_disable_driver();
    spindle_report_mcode_complete();
}

static void spindle_disable_driver_before_operation(void)
{
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
    spindle_disable_driver();
    LOG(LOG_LEVEL_INFO,
        "主轴停止，驱动器尚未进入运行使能，state=%d",
        spindle_driver.state);
}

static void spindle_disable_driver(void)
{
    spindle_driver.act_mode = DRV_MODE_NONE;
    Driver_Disable(&spindle_driver);
}

static bool spindle_is_rotating(void)
{
    switch (spl_context.spl_state)
    {
    case SPINDLE_CW:
    case SPINDLE_CCW:
        return true;
    case SPINDLE_STOP:
    default:
        return false;
    }
}

static void spindle_publish_status(void)
{
#ifdef DEBUG_MODE_ENABLED
    setVarChanInt(0, VAR_CHAN_S_CODE,
                  (int32_t)spl_context.vel_planner.current_vel, 0);
#else
    setVarChanInt(0, VAR_CHAN_S_CODE, spindle_driver.target_vel, 0);
#endif
}

static void spindle_report_mcode_complete(void)
{
    if (!spl_context.mcode_feedback)
    {
        spl_context.mcode_feedback = true;
        LOG(LOG_LEVEL_DEBUG,
            "主轴速度到达 %f",
            spl_context.vel_planner.current_vel);
    }
}
