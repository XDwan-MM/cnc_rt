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
#define SPINDLE_ALARM_ORIENT_TIMEOUT   (2100)
#define ALARM_RESULT_ALREADY_EXISTS    (-8)
#define SPINDLE_ORIENT_TIMEOUT_CYCLES  (10000U)

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
static void spindle_apply_deferred_start(void);

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
    bool release_orient_event;

    release_orient_event =
        (spl_context.spl_event == SPINDLE_EVENT_RELEASE_ORIENT);

    /*
     * 这是 1 ms 周期入口。处理顺序很重要：先锁定安全状态，再处理
     * 新事件，随后更新定向、驱动器控制和对外状态，避免本周期刚检测
     * 到急停又被后续的普通启动逻辑覆盖。
     */
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        (spindle_driver.status_word & 0x0008U) != 0U)
    {
        // printg("85\n");
        /* 急停解除后必须等待新的 M3/M4，不能恢复急停前的旧速度。 */
        spl_context.reset_requires_start = true;
        spl_context.start_request_pending = false;
        spl_context.orient_request_pending = false;
        spl_context.orient_release_pending = false;
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
        spl_context.orient_elapsed_cycles = 0U;
        spl_context.orient_hold_stop = false;
        spl_context.m19_active = false;
        spl_context.deferred_start_pending = false;
        spl_orient_abort();
    }

    if ((rt_data->nc_event == SYS_EVENT_HMI_RESET) ||
        (nc_reset != NULL && nc_reset->is_resetting))
    {
        spl_context.reset_requires_start = true;
        // printg("103\n");

        /*
         * 复位必须能够中止正在进行或已经完成的定向。PLC 的 G70.6
         * 下降沿通常会同时送入 RELEASE_ORIENT，但复位事件优先级更高
         * 时不能让下面的 STOP 覆盖该释放请求；没有下降沿时也要有
         * 兜底释放，避免状态永久停在 SPINDLE_ORIENT_DONE。
         */
        if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
            spl_context.spl_event == SPINDLE_EVENT_ORIENT ||
            spl_context.orient_request_pending)
        {
            // printg("115\n");
            spl_context.spl_event = SPINDLE_EVENT_RELEASE_ORIENT;
            release_orient_event = true;
        }
        /* 复位结束当前 M19 握手，新的 M3/M4 可以重新取得主轴控制权。 */
        spl_context.orient_hold_stop = false;
        spl_context.m19_active = false;
    }

    if (spindle_stop_is_required(&spl_context) && !release_orient_event)
    {
        // printg("126\n");
        spl_context.spl_event = SPINDLE_EVENT_STOP;
    }
    else if (!release_orient_event)
    {
        /* 复位后的旧 CW/CCW 事件由下面的专用过滤逻辑处理。 */
        spindle_update_command_event();
    }

    if (spl_context.reset_requires_start &&
        !spl_context.start_request_pending &&
        (spl_context.spl_event == SPINDLE_EVENT_CW ||
         spl_context.spl_event == SPINDLE_EVENT_CCW))
    {
        // printg("146\n");
        /* 丢弃复位前遗留的 PLC/方向边沿，直到新的 M3/M4 到来。 */
        spl_context.spl_event = SPINDLE_EVENT_NONE;
    }

    /* 事件处理只负责改变逻辑/驱动器目标；定向完成和速度规划在后续阶段更新。 */
    spindle_process_event(&spl_context);
    /* 直接读取 driver.c 的定向状态，并映射到 CNC/PLC 的 orient_phase。 */
    spindle_update_orient_state();
    /* 根据安全状态、定向状态和速度状态决定使能、模式及 60FF 目标值。 */
    spindle_update_driver_control();
    /* 将软件速度目标发布到 HMI 变量，供显示和诊断使用。 */
    spindle_publish_status();
}

/**
 * @brief 初始化主轴上下文中的面板启动速度。
 *
 * 其余成员由全局静态存储区保证初值为零；运行过程中，CNC/PLC 事件
 * 再分别填充 command_speed、command_state 和各类握手标志。
 */
void spl_context_init(spindle_context_t* spl_ctx)
{
    spl_ctx->mcp_target_speed = rt_data->axis_data[AXIS_SP].spl_default_speed;
}

/* ==========================================================================
 * 3. 内部函数定义
 * ========================================================================== */
/* CNC M3/M4/M5 只更新目标数据；目标发生变化时在控制层生成事件。 */
static void spindle_update_command_event(void)
{
    /*
     * 此函数不直接启动驱动器，只比较“期望目标”和规划器目标，生成
     * CW/CCW/STOP 事件。定向期间禁止执行，防止旧的 S 值抢回控制权。
     */
    /* 定向流程拥有主轴控制权，不能被旧的 M3/M4 目标重新启动。 */
    if (spl_context.orient_request_pending ||
        spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
        spl_context.orient_hold_stop)
    {
        // // printg("187****\n");
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

static void spindle_process_event(spindle_context_t *context)
{
    static bool stop_log_reported = false;

    /* spl_event 是单槽事件；本函数每周期消费一次并在末尾清空。 */
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
        spindle_handle_stop(context);
        if (spl_orient_is_done())
        {
            /* 允许重复执行 M19：先清除上一次完成状态再重新请求。 */
            if (!spl_orient_release())
            {
                context->orient_phase = SPINDLE_ORIENT_DONE;
                context->orient_release_pending = true;
                break;
            }
        }
        if (spl_orient_request())
        {
            context->orient_phase = SPINDLE_ORIENT_RUNNING;
            context->orient_elapsed_cycles = 0U;
        }
        else
        {
            // printg("**********************************************************\n");
            /* 请求失败时不能伪装成 RUNNING，否则状态机会永久等待 DONE。 */
            context->orient_phase = SPINDLE_ORIENT_IDLE;
            context->orient_release_pending = false;
            context->orient_hold_stop = true;
            context->reset_requires_start = true;
            context->start_request_pending = false;
            context->command_speed = 0.0;
        }
        // printg("context->orient_phase %d\n", context->orient_phase);
        // Ethercat_Driver_Write_Controlword(&spindle_driver, 0x040F);
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        /*
         * 释放既清除底层 0x040F 定向请求，也让普通主轴状态机重新接管。
         * 必须先清除 pending，避免本周期后续逻辑重新提交定向请求。
         */
        context->orient_request_pending = false;
        context->orient_release_pending = false;
        /* 释放/复位中止当前 M19，后续不应再为它产生完成反馈。 */
        context->m19_active = false;
        context->spl_state = SPINDLE_STOP;
        context->vel_planner.enable = false;
        context->vel_planner.target_vel = 0.0;
        Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
        if (spl_orient_release())
        {
            context->orient_phase = SPINDLE_ORIENT_RELEASING;
            context->orient_release_pending = false;
            context->orient_elapsed_cycles = 0U;
        }
        else
        {
            /* 驱动器拒绝释放时保留阶段，后续周期继续重试。 */
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
    /* 安全条件优先级最高：急停/Fault 时立即清零目标并请求下使能。 */
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

    /* 模式切换期间不允许速度目标继续加速，避免覆盖模式切换状态机。 */
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
    /* 定向控制字由 driver.c 维护，本模块不能在此期间写普通速度/下使能。 */
    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE)
    {
        if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING)
        {
            /* 释放阶段每周期维持下使能请求，直到状态字确认完成。 */
            Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
            Driver_Disable(&spindle_driver);
        }

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
    /*
     * 仅在驱动器处于 CSV 且允许速度控制时调用。规划器先计算 current_vel，
     * 再写入 HAL 缓存；EtherCAT 主循环负责把缓存发送到 60FF。
     */
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
    
    // // printg("Hal_Driver_Set_Target_Vel调用 target_vel: %d act_mode %d state: %d\n", spindle_driver.target_vel, spindle_driver.act_mode, spindle_driver.state);

    spindle_finish_stop_if_ready();
}

static bool spindle_stop_is_required(const spindle_context_t *context)
{
    /* 汇总所有必须停止主轴的原因，供周期入口统一仲裁。 */
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
    /*
     * 停止事件只撤销速度目标。普通 M5 保持驱动器可受控减速，急停/Fault
     * 则直接请求下使能；最终是否完成由 spindle_finish_stop_if_ready 判断。
     */
    context->spl_state = SPINDLE_STOP;
    context->vel_planner.enable = false;
    context->vel_planner.target_vel = 0.0;

    /*
     * 急停或驱动器 Fault 时不能请求重新使能：驱动器已经无法执行
     * 正常减速，释放急停后也必须保持下使能，防止旧目标速度恢复。
     * 普通 M5 则保持 CSV 模式并允许驱动器执行受控减速。
     */
    // if (!rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] &&
    //     spindle_driver.state != DRV_STATE_FAULT &&
    //     (spindle_driver.status_word & 0x0008U) == 0U &&
    //     spindle_driver.act_mode != DRV_MODE_NONE)
    if (!rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] &&
        spindle_driver.state != DRV_STATE_FAULT &&
        (spindle_driver.status_word & 0x0008U) == 0U)
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
    /* 校验速度后切换逻辑方向、选择 CSV 模式，并允许驱动器重新使能。 */
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
    /* 防止没有有效 S 值时误启动主轴；报警只在首次出现时记录。 */
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
    /* G33.7 选择 PLC 速度，否则使用 CNC M3/M4 的 S 值。 */
    if (context->speed_source == SPINDLE_SPEED_SOURCE_PLC)
    {
        return context->plc_speed;
    }

    return context->command_speed;
}

static double spindle_calculate_target_velocity(const spindle_context_t *context)
{
    /* 应用主轴倍率和最大转速限制，并按 CW/CCW 输出带符号速度。 */
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
    /* 软件规划器到位后仍需结合 606C 实际反馈，避免提前产生 FIN。 */
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
    /* 到速必须同时满足 CSV、Operation Enabled、非零和误差带条件。 */
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
     * 把 spl_orient.c 的底层完成标志同步到本文件的阶段状态，并在 M19
     * 完成时产生代码握手。释放请求可能在完成前到达，因此单独保留标志。
     */
    /* WAIT_ZERO 仅兼容其他可能的定向入口；当前入口直接进入 RUNNING。 */
    if (spl_context.orient_request_pending)
    {
        if (spl_orient_is_done())
        {
            /* 防止上一次定向状态残留。 */
            if (!spl_orient_release())
            {
                return;
            }
        }
        /* 兼容其他调用方：收到请求后也立即发送 0x040F。 */
        if (spl_orient_request())
        {
            spl_context.orient_request_pending = false;
            spl_context.orient_phase = SPINDLE_ORIENT_RUNNING;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_RUNNING)
    {
        if (spl_orient_is_done())
        {
            spl_context.orient_phase = SPINDLE_ORIENT_DONE;
            spl_context.orient_elapsed_cycles = 0U;
        }
        else if (++spl_context.orient_elapsed_cycles >=
                 SPINDLE_ORIENT_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR, "主轴定向超时，自动请求释放");
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            spl_context.m19_active = false;
            if (spl_orient_release())
            {
                spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
                spl_context.orient_release_pending = false;
                spl_context.orient_elapsed_cycles = 0U;
                spl_context.orient_hold_stop = true;
                spl_context.reset_requires_start = true;
                spl_context.start_request_pending = false;
            }
            else
            {
                spl_context.orient_phase = SPINDLE_ORIENT_DONE;
                spl_context.orient_release_pending = true;
            }
        }
    }

    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE &&
        spl_context.orient_release_pending)
    {
        if (spl_orient_release())
        {
            spl_context.orient_release_pending = false;
            spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_IDLE &&
        spl_context.orient_release_pending)
    {
        if (spl_orient_release())
        {
            spl_context.orient_release_pending = false;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING &&
        Driver_Is_Switched_On_Disabled(&spindle_driver))
    {
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
        spl_context.orient_hold_stop = false;
        spindle_apply_deferred_start();
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING &&
        spindle_driver.state != DRV_STATE_SWITCHED_ON_DISABLE)
    {
        if (++spl_context.orient_elapsed_cycles >=
            SPINDLE_ORIENT_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR,
                "主轴定向释放下使能超时：state=%d target_state=%d "
                "status_word=0x%04X orient_state=%d mode_switch_state=%d "
                "actual_vel=%d",
                spindle_driver.state,
                spindle_driver.target_state,
                spindle_driver.status_word,
                spindle_driver.orient_state,
                spindle_driver.mode_switch_state,
                spindle_driver.actual_vel);
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
            spl_context.orient_hold_stop = true;
            spl_context.reset_requires_start = true;
            spl_context.start_request_pending = false;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_DONE &&
        spl_context.m19_active)
    {
        spl_context.m19_active = false;
        spindle_report_mcode_complete();
    }
    // // printf("spl_context.orient_phase %d spl_context.orient_request_pending %d\n", spl_context.orient_phase, spl_context.orient_request_pending);
}

static void spindle_finish_stop_if_ready(void)
{
    /* 只有规划器速度和驱动器实际速度都为零，才允许完成停止握手。 */
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

static void spindle_apply_deferred_start(void)
{
    if (!spl_context.deferred_start_pending)
    {
        return;
    }

    spl_context.command_state = spl_context.deferred_start_state;
    spl_context.command_speed = spl_context.deferred_start_speed;
    spl_context.start_request_pending = true;
    spl_context.deferred_start_pending = false;
}

static void spindle_disable_driver_before_operation(void)
{
    /* 驱动器尚未进入 Operation Enabled 时，不能做受控减速，只能清零并下使能。 */
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
    /* 逻辑方向用于决定规划器是否继续输出目标速度。 */
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
    /* DEBUG 模式显示规划器速度，实时模式显示实际写入驱动器的目标速度。 */
#ifdef DEBUG_MODE_ENABLED
    setVarChanInt(0, VAR_CHAN_S_CODE,
                  (int32_t)spl_context.vel_planner.current_vel, 0);
#else
    setVarChanInt(0, VAR_CHAN_S_CODE, spindle_driver.target_vel, 0);
#endif
}

static void spindle_report_mcode_complete(void)
{
    /* FIN/完成反馈只允许从未完成变为完成一次，避免重复触发上层握手。 */
    if (!spl_context.mcode_feedback)
    {
        spl_context.mcode_feedback = true;
        // LOG(LOG_LEVEL_DEBUG,
        //     "主轴速度到达 %f",
        //     spl_context.vel_planner.current_vel);
    }
}
