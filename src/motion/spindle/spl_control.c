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
#define SPINDLE_ACTUAL_SPEED_EPSILON   (1.0)
#define SPINDLE_SPEED_ARRIVAL_TOL      (0.05)
#define SPINDLE_SPEED_ARRIVAL_MIN_TOL  (5.0)
#define SPINDLE_ALARM_MISSING_SPEED    (20)
#define ALARM_RESULT_ALREADY_EXISTS    (-8)

spindle_context_t spl_context;

/*
 * 主轴控制的职责边界：
 * 1. 本文件只维护 CNC/PLC 的主轴逻辑状态和速度规划；
 * 2. 速度目标通过 hal_driver_t->target_vel 交给驱动器 HAL；
 * 3. 驱动器实际使能、模式和状态字由 driver.c 状态机维护；
 * 4. M3/M4 的 FIN 只能在真实驱动器到速后产生，不能只依据软件规划器；
 * 5. M19 定向由驱动器收到 0x040F 后自行减速，定向完成依据状态字 bit10。
 */

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
static bool spindle_is_velocity_arrived(void);
static void spindle_finish_stop_if_ready(void);
static void spindle_disable_driver(void);
static void spindle_disable_driver_before_operation(void);
static bool spindle_is_rotating(void);
static void spindle_publish_status(void);
static void spindle_report_mcode_complete(void);
static void spindle_update_orient_state(void);

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
    /*
     * 这是 1 ms 周期入口。处理顺序很重要：先锁定安全状态，再处理
     * 新事件，随后更新定向、驱动器控制和对外状态，避免本周期刚检测
     * 到急停又被后续的普通启动逻辑覆盖。
     */
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        spindle_driver.state == DRV_STATE_FAULT ||
        (spindle_driver.status_word & 0x0008U) != 0U)
    {
        /* 急停解除后必须等待新的 M3/M4，不能恢复急停前的旧速度。 */
        spl_context.reset_requires_start = true;
        spl_context.start_request_pending = false;
    }

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
    spindle_update_orient_state();
    spindle_update_driver_control();
    spindle_publish_status();
}

void spl_context_init(spindle_context_t* spl_ctx)
{
    spl_ctx->mcp_target_speed = rt_data->axis_data[AXIS_SP].spl_default_speed;
}

/* CNC M3/M4/M5 只更新目标数据；目标发生变化时在控制层生成事件。 */
static void spindle_update_command_event(void)
{
    /* 定向流程拥有主轴控制权，不能被旧的 M3/M4 目标重新启动。 */
    if (spl_context.orient_request_pending ||
        spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
        spl_context.orient_hold_stop)
    {
        return;
    }

    /* command_speed 与 plc_speed 二选一，具体来源由 speed_source 决定。 */
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
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending || context->orient_hold_stop)
            break;
        spindle_handle_rotation(context, SPINDLE_CW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_CCW:
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending || context->orient_hold_stop)
            break;
        spindle_handle_rotation(context, SPINDLE_CCW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_ORIENT:
        /*
         * 定向不是普通的 M5 停止：驱动器收到 0x040F 后会自行减速并
         * 完成定位。因此这里立即提交定向请求，不等待软件速度规划器
         * 归零；否则高速主轴可能永远等不到本地“零速”条件。
         */
        context->orient_request_pending = false;
        context->orient_release_pending = false;
        context->orient_phase = SPINDLE_ORIENT_RUNNING;
        spindle_handle_stop(context);
        if (spl_orient_is_done())
        {
            /* 允许重复执行 M19：先清除上一次完成状态再重新请求。 */
            spl_orient_release();
        }
        spl_orient_request();
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        if (context->orient_phase == SPINDLE_ORIENT_DONE)
        {
            spl_orient_release();
            context->orient_phase = SPINDLE_ORIENT_IDLE;
            context->orient_release_pending = false;
        }
        else if (context->orient_phase != SPINDLE_ORIENT_IDLE)
        {
            context->orient_release_pending = true;
        }
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

    /*
     * 急停或驱动器 Fault 时，驱动器已经不能执行受控减速。
     * 此时必须立即清掉软件速度规划和目标速度，并保持下使能请求；
     * 不能再走 Driver_Enable()，否则急停释放后会把旧目标速度带回来。
     */
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        spindle_driver.state == DRV_STATE_FAULT ||
        (spindle_driver.status_word & 0x0008U) != 0U)
    {
        spl_context.spl_state = SPINDLE_STOP;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        spl_context.vel_planner.current_vel = 0.0;
        spl_context.vel_planner.active = false;
        /* 安全停机不是“到速”：禁止沿用急停前的 ARRIVED/FIN 状态。 */
        spl_context.spl_vel_state = SPINDLE_VEL_ZERO;
        spindle_driver.target_vel = 0;
        Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
        Driver_Disable(&spindle_driver);
        return;
    }

    if (Driver_Is_Mode_Switching(&spindle_driver))
    {
        /*
         * 模式切换期间禁止继续加速。若当前源模式是 CSV，仍让规划器
         * 输出受控减速速度；切换到其他模式时则直接保持速度目标为零。
         */
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        if (spindle_driver.act_mode == DRV_MODE_CSV)
            spindle_update_velocity();
        else
            spindle_driver.target_vel = 0;
        return;
    }

    /* 定向期间由驱动器自行减速，不能被普通停止流程下使能或改写。 */
    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE)
    {
        /*
         * 定向期间 0x040F 由 driver.c 持续发送。这里不能调用普通速度
         * 更新或 Driver_Disable，否则会覆盖定向控制字或提前撤销使能。
         */
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
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
            /*
             * 已经不在 Operation Enabled 时，不能再做受控减速；只确保
             * 目标速度为零并等待实际速度归零，然后完成停止握手。
             */
            spindle_disable_driver_before_operation();
            /* 即使驱动器已经处于下使能状态，也要完成停止到速握手。 */
            spindle_finish_stop_if_ready();
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
    /* enable=false 时，规划器按最大减速度将 current_vel 拉回零。 */
    planner->enable = spindle_is_rotating() && !Driver_Is_Mode_Switching(&spindle_driver);
    planner->target_vel = 0.0;

    if (planner->enable)
    {
        planner->target_vel = spindle_calculate_target_velocity(&spl_context);
    }

    spindle_update_planner_state(&spl_context);

    /*
     * 规划器输出是本周期的目标速度。这里只更新 HAL 缓存，真正写入
     * EtherCAT 60FF 由主站发送周期统一完成。
     */
    spindle_driver.target_vel = (int32_t)planner->current_vel;
    
    Hal_Driver_Set_Target_Vel(&spindle_driver, spindle_driver.target_vel);

    // Ethercat_Driver_Write_Target_Velocity(&spindle_driver, spindle_driver.target_vel);
    
    // printf("Hal_Driver_Set_Target_Vel调用 target_vel: %d act_mode %d state: %d\n", spindle_driver.target_vel, spindle_driver.act_mode, spindle_driver.state);

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

    /*
     * 急停或驱动器 Fault 时不能请求重新使能：驱动器已经无法执行
     * 正常减速，释放急停后也必须保持下使能，防止旧目标速度恢复。
     * 普通 M5 则保持 CSV 模式并允许驱动器执行受控减速。
     */
    if (!rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] &&
        spindle_driver.state != DRV_STATE_FAULT &&
        (spindle_driver.status_word & 0x0008U) == 0U &&
        spindle_driver.act_mode != DRV_MODE_NONE)
    {
        Driver_Enable(&spindle_driver);
    }
    else
    {
        Driver_Disable(&spindle_driver);
    }
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
    // 直接下使能
    // Ethercat_Driver_Write_Target_Velocity(&spindle_driver, 0);
    // Ethercat_Driver_Write_Controlword(&spindle_driver, 0x06);
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

    /*
     * 新 M3/M4 是急停/复位后的明确启动授权。清除 reset_requires_start
     * 后才允许 Driver_Enable，防止旧的 PLC 方向信号自动恢复主轴。
     */
    context->spl_state = state;
    if (context->reset_requires_start && !context->start_request_pending)
    {
        return;
    }
    context->reset_requires_start = false;
    context->start_request_pending = false;
    context->mcode_feedback = false;
    context->spl_vel_state = SPINDLE_VEL_RAMP;
    spindle_driver.act_mode = DRV_MODE_CSV;
    Driver_Enable(&spindle_driver);

    // LOG(LOG_LEVEL_INFO,
    //     "主轴收到%s转指令，目标速度=%f",
    //     state == SPINDLE_CW ? "正" : "反",
    //     spindle_calculate_target_velocity(context));
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
    const bool arrived = spl_vel_planner_update(&context->vel_planner, SPINDLE_CONTROL_PERIOD_SECONDS);

    if (!context->vel_planner.active)
    {
        context->spl_vel_state = SPINDLE_VEL_ZERO;
        return;
    }
    if (arrived)
    {
        /* 软件规划器到达目标不等于电机已经到速，必须继续检查 PDO 反馈。 */
        if (spindle_is_velocity_arrived())
        {
            context->spl_vel_state = SPINDLE_VEL_ARRIVED;
            spindle_report_mcode_complete();
        }
        else
        {
            context->spl_vel_state = SPINDLE_VEL_RAMP;
        }
        return;
    }

    context->spl_vel_state = SPINDLE_VEL_RAMP;
}

static bool spindle_is_velocity_arrived(void)
{
    const double target = spl_context.vel_planner.current_vel;
    const double actual = (double)spindle_driver.actual_vel;
    const double tolerance = fmax(SPINDLE_SPEED_ARRIVAL_MIN_TOL,
                                  fabs(target) * SPINDLE_SPEED_ARRIVAL_TOL);

    /*
     * 606C 实际速度和 60FF 目标速度必须来自同一 CSV 单位。除误差带外
     * 还要求实际速度非零，避免驱动器未使能或电机未转时误发 M3 FIN。
     */
    if (spindle_driver.act_mode != DRV_MODE_CSV ||
        spindle_driver.state != DRV_STATE_OPERATION_ENABLED)
    {
        return false;
    }

    if (fabs(target) <= SPINDLE_SPEED_EPSILON ||
        fabs(actual) <= SPINDLE_ACTUAL_SPEED_EPSILON)
    {
        return false;
    }

    return fabs(actual - target) <= tolerance;
}

static void spindle_update_orient_state(void)
{
    /*
     * WAIT_ZERO 仅兼容其他可能的定向入口；当前 M19/面板事件会直接
     * 进入 RUNNING。RUNNING 状态等待 spl_orient 模块检测到状态字 bit10。
     */
    if (spl_context.orient_phase == SPINDLE_ORIENT_WAIT_ZERO &&
        spl_context.orient_request_pending)
    {
        if (spl_orient_is_done())
        {
            /* 防止上一次定向状态残留。 */
            spl_orient_release();
        }
        /* 兼容其他调用方：收到请求后也立即发送 0x040F。 */
        spl_orient_request();
        spl_context.orient_request_pending = false;
        spl_context.orient_phase = SPINDLE_ORIENT_RUNNING;
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_RUNNING &&
        spl_orient_is_done())
    {
        spl_context.orient_phase = SPINDLE_ORIENT_DONE;
        if (spl_context.m19_active)
        {
            spl_context.m19_active = false;
            spindle_report_mcode_complete();
        }
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_DONE &&
        spl_context.orient_release_pending)
    {
        spl_orient_release();
        spl_context.orient_release_pending = false;
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
    }
}

static void spindle_finish_stop_if_ready(void)
{
    /*
     * 停止完成必须同时满足软件规划速度和驱动器实际速度都为零。
     * 仅清零 60FF 不代表电机已经停止，尤其在驱动器下使能或反馈延迟时。
     */
    if (spl_context.spl_state != SPINDLE_STOP)
    {
        return;
    }
    if (fabs(spl_context.vel_planner.current_vel) > SPINDLE_SPEED_EPSILON)
    {
        return;
    }
    if (fabs((double)spindle_driver.actual_vel) > SPINDLE_ACTUAL_ZERO_EPSILON)
    {
        return;
    }

    if (spindle_driver.act_mode != DRV_MODE_NONE)
    {
        Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
        spindle_disable_driver();
    }

    /* 先发布 ARRIVED，让 PLC 在本周期输出 F45.3；下一周期再回到 ZERO。 */
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spindle_report_mcode_complete();
}

static void spindle_disable_driver_before_operation(void)
{
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
    spindle_disable_driver();
    // LOG(LOG_LEVEL_INFO,
    //     "主轴停止，驱动器尚未进入运行使能，state=%d",
    //     spindle_driver.state);
}

static void spindle_disable_driver(void)
{
    /*
     * 停止只撤销驱动器使能，不清除实际工作模式。
     * 主轴正常运行模式为 CSV，停止后仍保持 6060/6061 = 0x09，
     * 后续收到 M3/M4 时可以直接重新使能。
     */
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
        // LOG(LOG_LEVEL_DEBUG,
        //     "主轴速度到达 %f",
        //     spl_context.vel_planner.current_vel);
    }
}
