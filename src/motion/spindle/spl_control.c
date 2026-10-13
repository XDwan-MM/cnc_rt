#include "spl_control.h"
#include "driver.h"
#include "rt_data.h"
#include "spl_orient.h"
#include "spl_vel_planner.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <parapi.h>
#include <regapi.h>

/* 周期单位为秒；速度及速度误差单位为 rpm；超时单位为控制周期（1 ms）。 */
#define SPINDLE_CONTROL_PERIOD_SECONDS (0.001)
#define SPINDLE_SPEED_EPSILON          (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON    (1.0)
#define SPINDLE_ACTUAL_SPEED_EPSILON   (1.0)
/* 到速允许误差取有效目标的 5% 与 5 rpm 中的较大值。 */
#define SPINDLE_SPEED_ARRIVAL_TOL      (0.05)
#define SPINDLE_SPEED_ARRIVAL_MIN_TOL  (5.0)
#define SPINDLE_ALARM_MISSING_SPEED    (20)
#define SPINDLE_ALARM_ORIENT_TIMEOUT   (2100)
#define ALARM_RESULT_ALREADY_EXISTS    (-8)
#define SPINDLE_ORIENT_TIMEOUT_CYCLES  (10000U)
#define SPINDLE_M29_MODE_TIMEOUT_CYCLES (10000U)

spindle_context_t spl_context;

/*
 * 主轴控制的职责边界：
 * 1. 本文件只维护 CNC/PLC 的主轴逻辑状态和速度规划；
 * 2. 速度目标通过 hal_driver_t->target_vel 交给驱动器 HAL；
 * 3. 驱动器实际使能、模式和状态字由 driver.c 状态机维护；
 * 4. 本文件确认主轴完成条件，PLC 产生 G4.3 FIN，auto_mst.c 推进指令；
 * 5. M19 定向由驱动器收到 0x040F 后自行减速，定向完成依据底层定向状态。
 *
 * 控制周期顺序：任务事件 -> spindle_control_update -> 自动指令 -> F 信号发布。
 * command_state/command_speed 表示最新请求，spl_state 表示已接管的运行方向。
 * vel_planner.current_vel 是软件规划输出，不是电机实际速度 actual_vel。
 *
 * 控制权与阶段：
 * - 普通旋转：CSV 速度规划负责 60FF；有效目标已应用倍率和限速。
 * - 定向：RUNNING -> DONE -> RELEASING -> IDLE；驱动器掌握速度控制权。
 *   释放确认时仅同步一次 actual_vel，再在同周期接管合法的新旋转请求。
 * - M29：ORIENTING -> RELEASING -> SWITCHING_CSP -> READY；READY 后保持
 *   固定位置。m29_active 表示辅助指令仍在握手，READY 可以延续到握手之后。
 *
 * 反馈约束：F45.3 查询必须验证当前请求；F45.1 由 PLC 发布层独立输出真实
 * 零速。M29 的 PLC FIN 锁存归 auto_mst.c 管理，本模块不操作 F76.3。
 *
 * 下面按“内部声明 / 公共接口 / 内部实现”组织；内部实现依次为安全仲裁、
 * 事件处理、驱动器控制、速度判定、定向与 M29、交接收尾和诊断。
 */

/* ==========================================================================
 * 1. 内部函数声明
 * ========================================================================== */

/* 安全仲裁与驱动器反馈。 */
static bool spindle_driver_has_fault(void);
static bool spindle_reset_is_requested(void);
static void spindle_handle_safety_requests(bool *release_orient_event);
static bool spindle_rotation_event_blocked(const spindle_context_t *context);
static bool spindle_csp_switch_is_complete(void);
static void spindle_handle_orient_request(spindle_context_t *context);
static void spindle_handle_orient_release(spindle_context_t *context);
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
static void spindle_update_m29(void);
static bool spindle_velocity_feedback_blocked(void);
static bool spindle_stop_is_complete(void);
static bool spindle_reset_or_estop_active(void);
static void spindle_log_m29_csp_trace(void);

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

    /* 先取消不再有效的请求，再仲裁本周期的命令。 */
    spindle_handle_safety_requests(&release_orient_event);

    if (spindle_stop_is_required(&spl_context) && !release_orient_event)
    {
        spl_context.spl_event = SPINDLE_EVENT_STOP;
    }
    else if (!release_orient_event)
    {
        /* 复位后的旧 CW/CCW 事件由下面的专用过滤逻辑处理。 */
        spindle_update_command_event();
    }

    const bool rotation_event = spl_context.spl_event == SPINDLE_EVENT_CW ||
                                spl_context.spl_event == SPINDLE_EVENT_CCW;
    const bool start_not_authorized = spl_context.reset_requires_start &&
                                     !spl_context.start_request_pending;
    if (start_not_authorized && rotation_event)
    {
        /* 丢弃复位前遗留的 PLC/方向边沿，直到新的 M3/M4 到来。 */
        spl_context.spl_event = SPINDLE_EVENT_NONE;
    }

    /* 事件处理只负责改变逻辑/驱动器目标；定向完成和速度规划在后续阶段更新。 */
    spindle_process_event(&spl_context);
    /* 直接读取 driver.c 的定向状态，并映射到 CNC/PLC 的 orient_phase。 */
    spindle_update_orient_state();
    /* M29 立即检查本周期更新后的定向阶段；这里不额外等待一个周期。 */
    spindle_update_m29();
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

/**
 * 接收新的 M29，取消旧的旋转请求并启动定向。
 * 本文件只管理 NC 动作；auto_mst.c 负责 F7.0/G4.3 握手和 FIN 锁存。
 * M29 不在这里置位或清零 F76.3，也不自行生成 PLC FIN。
 */
void spindle_request_m29(void)
{
    /* M29 本身是复位后的新主轴操作请求，允许重新取得驱动器使能。 */
    spl_context.reset_requires_start = false;
    spl_context.start_request_pending = true;
    spindle_clear_velocity_arrival();
    spl_context.deferred_start_pending = false;
    spl_context.m29_active = true;
    spl_context.m29_phase = SPINDLE_M29_ORIENTING;
    spl_context.m19_active = false;
    spl_context.orient_hold_stop = true;
    spl_context.mcode_feedback = false;
    spl_context.spl_event = SPINDLE_EVENT_ORIENT;
    spl_context.command_speed = 0.0;
    spl_context.command_state = SPINDLE_STOP;
    spl_context.start_request_pending = false;
}

/**
 * 查询当前 M29 的实时 NC 就绪条件，不读取或修改 PLC FIN。
 * READY 枚举不足以保证仍然就绪：还要检查模式切换结果、实际 CSP、
 * 软件使能状态和 PDO 状态字。位置误差仅用于诊断，不是此接口的门槛。
 */
bool spindle_m29_is_ready(void)
{
    if (spindle_stop_is_required(&spl_context))
    {
        return false;
    }
    if (!spl_context.m29_active || spl_context.reset_requires_start)
    {
        return false;
    }
    if (spl_context.m29_phase != SPINDLE_M29_READY)
    {
        return false;
    }
    if (!Driver_Is_Mode_Switch_Done(&spindle_driver))
    {
        return false;
    }
    if (spindle_driver.act_mode != DRV_MODE_CSP)
    {
        return false;
    }
    if (spindle_driver.state != DRV_STATE_OPERATION_ENABLED)
    {
        return false;
    }
    return (spindle_driver.status_word & 0x006FU) == 0x0027U;
}

/** 由自动指令层在双方完成后调用；结束握手但保留固定位置保持。 */
void spindle_complete_m29(void)
{
    if (!spindle_m29_is_ready())
    {
        return;
    }

    /* auto_mst.c 已确认本条 PLC FIN 与 NC 就绪同时成立。
     * 仅结束辅助指令握手，继续保持 CSP 使能和固定目标位置。
     * 后续 M03/M04 才解除 READY 并请求安全切回 CSV。 */
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_READY;
    spl_context.orient_hold_stop = true;
    spl_context.command_state = SPINDLE_STOP;
    spl_context.command_speed = 0.0;
    spl_context.mcode_feedback = false;
}

/** 新请求接收时立即撤销旧反馈，不能等到下一次主轴控制周期。 */
void spindle_clear_velocity_arrival(void)
{
    spl_context.mcode_feedback = false;
    spl_context.spl_vel_state = SPINDLE_VEL_RAMP;
    clearRegBit(REG_F, 45, 3);
}

/**
 * PLC 发布 F45.3 的唯一查询入口；本函数只读状态。
 * 内部完成标志是必要条件，但发布前仍重新验证当前控制权和实际反馈，
 * 防止新指令在主轴更新之后、F 信号发布之前到达时沿用上一条 ARRIVED。
 */
bool spindle_velocity_is_ready(void)
{
    if (spindle_velocity_feedback_blocked())
    {
        return false;
    }
    if (!spl_context.mcode_feedback)
    {
        return false;
    }
    if (spl_context.spl_vel_state != SPINDLE_VEL_ARRIVED)
    {
        return false;
    }
    if (spl_context.spl_state == SPINDLE_STOP)
    {
        return spindle_stop_is_complete();
    }
    return spindle_is_velocity_arrived();
}

/* 只对离散状态/有效目标变化记录快照，速度斜坡本身不触发逐周期日志。 */
void spindle_trace_status(void)
{
    static uint64_t cycle;
    static bool initialized;
    static unsigned last_flags;
    static double last_target;
    const char *mode_name;
    bool mf = false, fin = false;
    getRegBit(REG_F, 7, 0, &mf);
    getRegBit(REG_G, 4, 3, &fin);
    bool ready = false;
    getRegBit(REG_F, 45, 3, &ready);
    const double target = spindle_is_rotating() ?
        spindle_calculate_target_velocity(&spl_context) : 0.0;
    /* 比较位图只编码离散状态：0..2 定向，3..5 M29，6..7 待启动，
     * 8..10 为 F45.3/F7.0/G4.3。规划/实际速度变化不触发逐周期日志。 */
    const unsigned flags = (unsigned)spl_context.orient_phase |
        ((unsigned)spl_context.m29_phase << 3) |
        ((unsigned)spl_context.start_request_pending << 6) |
        ((unsigned)spl_context.deferred_start_pending << 7) |
        ((unsigned)ready << 8) | ((unsigned)mf << 9) | ((unsigned)fin << 10);
    switch (spindle_driver.act_mode)
    {
    case DRV_MODE_CSV:
        mode_name = "CSV";
        break;
    case DRV_MODE_CSP:
        mode_name = "CSP";
        break;
    default:
        mode_name = "NONE";
        break;
    }
    ++cycle;
    if (!initialized || flags != last_flags || target != last_target)
    {
        if (spindle_driver.act_mode == DRV_MODE_CSP)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴状态[%llu] mode=%s target_rpm=%.3f planned_rpm=%.3f "
                "actual_rpm=%d target_pos=%d actual_pos=%d position_error=%d "
                "orient=%d start_pending=%d deferred_start=%d "
                "F45.3_ready=%d F7.0_MF=%d G4.3_FIN=%d M29_phase=%d",
                (unsigned long long)cycle, mode_name, target,
                spl_context.vel_planner.current_vel, spindle_driver.actual_vel,
                spindle_driver.target_pos, spindle_driver.actual_pos,
                spindle_driver.target_pos - spindle_driver.actual_pos,
                spl_context.orient_phase, spl_context.start_request_pending,
                spl_context.deferred_start_pending, ready, mf, fin,
                spl_context.m29_phase);
        }
        else
        {
            LOG(LOG_LEVEL_INFO,
                "主轴状态[%llu] mode=%s target_rpm=%.3f planned_rpm=%.3f "
                "actual_rpm=%d orient=%d start_pending=%d deferred_start=%d "
                "F45.3_ready=%d F7.0_MF=%d G4.3_FIN=%d M29_phase=%d",
                (unsigned long long)cycle, mode_name, target,
                spl_context.vel_planner.current_vel, spindle_driver.actual_vel,
                spl_context.orient_phase, spl_context.start_request_pending,
                spl_context.deferred_start_pending, ready, mf, fin,
                spl_context.m29_phase);
        }
    }
    initialized = true;
    last_flags = flags;
    last_target = target;
}

/* ==========================================================================
 * 3. 内部函数定义
 * ========================================================================== */
/* 软件状态与 PDO 故障位均检查，避免任一反馈更新滞后时漏掉故障。 */
static bool spindle_driver_has_fault(void)
{
    const bool state_fault = spindle_driver.state == DRV_STATE_FAULT;
    const bool status_fault = (spindle_driver.status_word & 0x0008U) != 0U;
    return state_fault || status_fault;
}

/* RESET 事件只持续一个周期；复位流程标志可能跨越多个周期。 */
static bool spindle_reset_is_requested(void)
{
    const bool reset_event = rt_data->nc_event == SYS_EVENT_HMI_RESET;
    const bool reset_in_progress = nc_reset != NULL && nc_reset->is_resetting;
    return reset_event || reset_in_progress;
}

/* 用于普通旋转事件的入口屏蔽；释放确认后由交接函数直接执行新启动。 */
static bool spindle_rotation_event_blocked(const spindle_context_t *context)
{
    if (context->orient_phase != SPINDLE_ORIENT_IDLE)
    {
        return true;
    }
    if (context->orient_request_pending)
    {
        return true;
    }
    return context->orient_hold_stop && !context->start_request_pending;
}

/* 只检查 M29 的 CSP 切换完成条件；实时就绪接口另检查安全条件和状态字。 */
static bool spindle_csp_switch_is_complete(void)
{
    if (!Driver_Is_Mode_Switch_Done(&spindle_driver))
    {
        return false;
    }
    if (spindle_driver.act_mode != DRV_MODE_CSP)
    {
        return false;
    }
    return spindle_driver.state == DRV_STATE_OPERATION_ENABLED;
}

/**
 * 处理急停、驱动器故障、复位及报警，清除旧启动授权。
 * @param release_orient_event 复位需要释放定向时置为 true；原有释放请求保留。
 *
 * 急停/故障直接中止底层定向；普通复位仍通过 RELEASE_ORIENT 收回控制权。
 * 两者不是互斥分支：同周期发生时，按原顺序先处理故障，再处理复位。
 */
static void spindle_handle_safety_requests(bool *release_orient_event)
{
    /*
     * 这是 1 ms 周期入口。处理顺序很重要：先锁定安全状态，再处理
     * 新事件，随后更新定向、驱动器控制和对外状态，避免本周期刚检测
     * 到急停又被后续的普通启动逻辑覆盖。
     */
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] || spindle_driver_has_fault())
    {
        /* 急停解除后必须等待新的 M3/M4，不能恢复急停前的旧速度。 */
        spl_context.reset_requires_start = true;
        spindle_clear_velocity_arrival();
        spl_context.start_request_pending = false;
        spl_context.orient_request_pending = false;
        spl_context.orient_release_pending = false;
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
        spl_context.orient_elapsed_cycles = 0U;
        spl_context.orient_hold_stop = false;
        spl_context.m19_active = false;
        spl_context.m29_active = false;
        spl_context.m29_phase = SPINDLE_M29_IDLE;
        spl_context.csv_switch_pending = false;
        spl_context.deferred_start_pending = false;
        /* 安全事件取消遗留模式切换，避免驱动器周期函数继续执行旧
         * 切换并在超时后返回 HAL_DRIVER_ERROR。 */
        if (Driver_Is_Mode_Switching(&spindle_driver))
        {
            spindle_driver.mode_switch_state = DRV_MODE_SWITCH_IDLE;
            spindle_driver.mode_switch_result = HAL_DRIVER_SUCCESS;
            spindle_driver.mode_switch_cycles = 0U;
        }
        spl_orient_abort();
    }

    if (spindle_reset_is_requested())
    {
        spl_context.reset_requires_start = true;
        spl_context.start_request_pending = false;
        spl_context.deferred_start_pending = false;
        spindle_clear_velocity_arrival();

        /*
         * 复位必须能够中止正在进行或已经完成的定向。PLC 的 G70.6
         * 下降沿通常会同时送入 RELEASE_ORIENT，但复位事件优先级更高
         * 时不能让下面的 STOP 覆盖该释放请求；没有下降沿时也要有
         * 兜底释放，避免状态永久停在 SPINDLE_ORIENT_DONE。
         */
        const bool orient_in_progress = spl_context.orient_phase != SPINDLE_ORIENT_IDLE;
        const bool orient_event = spl_context.spl_event == SPINDLE_EVENT_ORIENT;
        const bool orient_waiting = orient_event || spl_context.orient_request_pending;
        if (orient_in_progress || orient_waiting)
        {
            spl_context.spl_event = SPINDLE_EVENT_RELEASE_ORIENT;
            *release_orient_event = true;
        }
        /* 复位结束当前 M19 握手，新的 M3/M4 可以重新取得主轴控制权。 */
        spl_context.orient_hold_stop = false;
        spl_context.m19_active = false;
        spl_context.m29_active = false;
        spl_context.m29_phase = SPINDLE_M29_IDLE;
        spl_context.csv_switch_pending = false;
        /* HMI RESET 同样必须取消所有未完成的模式切换；否则周期驱动
         * 会在普通停止逻辑之前继续执行旧切换状态。 */
        spindle_driver.mode_switch_state = DRV_MODE_SWITCH_IDLE;
        spindle_driver.mode_switch_result = HAL_DRIVER_SUCCESS;
        spindle_driver.mode_switch_cycles = 0U;
    }

    if (rt_data->sys_state[SYS_IS_ALARMING])
    {
        spl_context.reset_requires_start = true;
        spl_context.start_request_pending = false;
        spl_context.deferred_start_pending = false;
        spindle_clear_velocity_arrival();
    }

}

/*
 * 命令仲裁：把 CNC/PLC 写入的目标转换成一个单周期事件。
 * 事件只存放在 spl_event 中，由 spindle_process_event() 消费。
 */
static void spindle_update_command_event(void)
{
    /* 单槽显式事件优先；定向和模式切换期间不能重新生成旋转事件。 */
    if (spl_context.spl_event != SPINDLE_EVENT_NONE)
    {
        return;
    }
    /* 请求与释放的等待窗口同样属于定向控制权，尚不能恢复 CSV。 */
    if (spl_context.orient_request_pending || spl_context.orient_release_pending)
    {
        return;
    }
    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE)
    {
        return;
    }
    if (spl_context.m29_active || spl_context.csv_switch_pending)
    {
        return;
    }
    if (Driver_Is_Mode_Switching(&spindle_driver))
    {
        return;
    }
    if (spl_context.orient_hold_stop && !spl_context.start_request_pending)
    {
        return;
    }
    if (spl_context.reset_requires_start && !spl_context.start_request_pending)
    {
        return;
    }

    const spindle_state_t direction = spl_context.command_state;
    spindle_context_t requested = spl_context;
    requested.spl_state = direction;
    const double signed_target = direction == SPINDLE_STOP ? 0.0 :
        spindle_calculate_target_velocity(&requested);
    const double target_speed = fabs(spindle_get_selected_speed(&spl_context));
    const bool same_target = fabs(signed_target - spl_context.vel_planner.target_vel)
        <= SPINDLE_SPEED_EPSILON;
    const bool same_direction = direction == spl_context.spl_state;
    /* 全新同方向/同 S 指令仍须处理，不能仅凭目标数值相等跳过接管。 */
    if (!spl_context.start_request_pending && same_target && same_direction)
    {
        return;
    }

    if (direction == SPINDLE_STOP || target_speed <= SPINDLE_SPEED_EPSILON)
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

/*
 * 事件执行：这里只处理状态切换和驱动器请求，不直接判断到速。
 * 速度规划与 FIN 判定分别在后续控制阶段完成。
 */
static void spindle_process_event(spindle_context_t *context)
{
    static bool stop_log_reported = false;

    const bool nc_owns_orientation = context->orient_phase != SPINDLE_ORIENT_IDLE ||
                                     context->orient_request_pending;
    const bool driver_owns_orientation = spindle_driver.orient_state != DRV_ORIENT_IDLE;
    const bool orient_event = context->spl_event == SPINDLE_EVENT_ORIENT;
    const bool stop_blocked = nc_owns_orientation || driver_owns_orientation || orient_event;

    /* spl_event 是单槽事件；本函数每周期消费一次并在末尾清空。 */
    switch (context->spl_event)
    {
    case SPINDLE_EVENT_STOP:
        /* G29.6 的停止边沿可能紧跟 G70.6 定向上升沿到达。定向请求
         * 已经接管主轴后，普通 STOP 不能撤销 Operation Enabled，否则
         * driver.c 永远等不到 0x27，最终表现为定向超时。释放定向或
         * 复位事件使用专用 RELEASE_ORIENT 路径。 */
        if (stop_blocked)
        {
            break;
        }
        if (context->m29_phase == SPINDLE_M29_READY)
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
        }
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
        if (spindle_rotation_event_blocked(context))
        {
            break;
        }
        spindle_handle_rotation(context, SPINDLE_CW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_CCW:
        if (spindle_rotation_event_blocked(context))
        {
            break;
        }
        spindle_handle_rotation(context, SPINDLE_CCW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_ORIENT:
        spindle_handle_orient_request(context);
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        spindle_handle_orient_release(context);
        break;
    case SPINDLE_EVENT_NONE:
    default:
        break;
    }

    context->spl_event = SPINDLE_EVENT_NONE;
}

/* 定向启动只提交驱动器请求，完成情况由 spindle_update_orient_state 跟踪。 */
static void spindle_handle_orient_request(spindle_context_t *context)
{
    spindle_clear_velocity_arrival();
    context->deferred_start_pending = false;
    context->command_state = SPINDLE_STOP;
    context->command_speed = 0.0;
    /*
     * 定向不是普通的 M5 停止：驱动器收到 0x040F 后会自行减速并
     * 完成定位。因此这里立即提交定向请求，不等待软件速度规划器
     * 归零；否则高速主轴可能永远等不到本地“零速”条件。
     */
    /* 显式 M19/M29 定向请求可以解除复位后的“等待新启动”锁。 */
    context->reset_requires_start = false;
    context->start_request_pending = false;
    context->orient_request_pending = false;
    context->orient_release_pending = false;
    /* 定向需先请求 Operation Enabled，不能复用普通停止处理；
     * 普通停止可能发起 CSP -> CSV，把定向所需的使能目标覆盖成下使能。 */
    context->spl_state = SPINDLE_STOP;
    context->vel_planner.enable = false;
    context->vel_planner.target_vel = 0.0;
    spindle_driver.target_vel = 0;
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
    Driver_Enable(&spindle_driver);
    if (spl_orient_is_done())
    {
        /* 允许重复执行 M19：先清除上一次完成状态再重新请求。 */
        if (!spl_orient_release())
        {
            context->orient_phase = SPINDLE_ORIENT_RELEASING;
            context->orient_release_pending = true;
            context->m19_active = false;
            return;
        }
    }
    if (spl_orient_request())
    {
        context->orient_phase = SPINDLE_ORIENT_RUNNING;
        context->orient_elapsed_cycles = 0U;
    }
    else
    {
        /* 请求失败时不能伪装成 RUNNING，否则状态机会永久等待 DONE。 */
        context->orient_phase = SPINDLE_ORIENT_IDLE;
        context->orient_release_pending = false;
        context->orient_hold_stop = true;
        context->m19_active = false;
        context->reset_requires_start = true;
        context->start_request_pending = false;
        context->command_speed = 0.0;
    }
}

/* 释放不等于已下使能；保留 RELEASING，等待状态字确认后才交接速度控制。 */
static void spindle_handle_orient_release(spindle_context_t *context)
{
    spindle_clear_velocity_arrival();
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
}

/*
 * 驱动器控制状态机：安全停机、模式切换、定向、M29 CSP 保持和普通 CSV
 * 速度控制按优先级依次处理。每个分支完成自己的工作后立即返回，避免
 * 同一个周期同时执行互相冲突的控制字请求。
 */
static void spindle_update_driver_control(void)
{
    bool velocity_control_ready = false;

    /*
     * 急停或驱动器 Fault 时，驱动器已经不能执行受控减速。
     * 此时必须立即清掉软件速度规划和目标速度，并保持下使能请求；
     * 不能再走 Driver_Enable()，否则急停释放后会把旧目标速度带回来。
     */
    /* 安全条件优先级最高：急停/Fault 时立即清零目标并请求下使能。 */
    if (spindle_reset_or_estop_active() || spindle_driver_has_fault())
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
        /* 急停/Fault 不等待 CSP->CSV。取消遗留切换并立即请求下使能，
         * 否则 Driver_Cyclic_Process 仍可能继续旧切换直到超时。 */
        if (Driver_Is_Mode_Switching(&spindle_driver))
        {
            spindle_driver.mode_switch_state = DRV_MODE_SWITCH_IDLE;
            spindle_driver.mode_switch_result = HAL_DRIVER_SUCCESS;
            spindle_driver.mode_switch_cycles = 0U;
        }
        spl_context.csv_switch_pending = false;
        const bool csv_already_requested = spindle_driver.req_mode == DRV_MODE_CSV &&
                                           Driver_Is_Mode_Switch_Done(&spindle_driver);
        const bool csp_needs_csv = spindle_driver.act_mode == DRV_MODE_CSP &&
                                   !csv_already_requested;
        const bool disabled_reset = spindle_reset_or_estop_active() &&
                                    Driver_Is_Ready_to_Switch_On(&spindle_driver);
        if (disabled_reset && csp_needs_csv)
        {
            if (Driver_Switch_Mode_While_Disabled(&spindle_driver,
                                                   DRV_MODE_CSV) !=
                HAL_DRIVER_SUCCESS)
            {
                LOG(LOG_LEVEL_ERR,
                    "复位/急停后下使能状态切换CSV失败：status_word=0x%04X",
                    spindle_driver.status_word);
            }
        }
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
        {
            spindle_update_velocity();
        }
        else
        {
            spindle_driver.target_vel = 0;
        }
        return;
    }

    if (spl_context.m29_phase == SPINDLE_M29_ERROR)
    {
        spl_context.spl_state = SPINDLE_STOP;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        spindle_driver.target_vel = 0;
        Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
        if (spindle_driver.act_mode == DRV_MODE_CSP)
        {
            spindle_driver.mode_switch_state = DRV_MODE_SWITCH_IDLE;
            spindle_driver.mode_switch_result = HAL_DRIVER_SUCCESS;
            spindle_driver.mode_switch_cycles = 0U;
        }
        spl_context.csv_switch_pending = false;
        Driver_Disable(&spindle_driver);
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

    if (spl_context.m29_phase == SPINDLE_M29_READY)
    {
        spl_context.spl_state = SPINDLE_STOP;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        /*
         * M29 完成后 CSP 必须保持“定向完成瞬间”的固定位置。
         * 不能在每个周期把目标位置追写成 actual_pos，否则目标会跟着
         * 电机一起移动，驱动器永远不会真正锁定定向位置；复位或后续
         * 模式切换时也会表现为主轴继续正转。
         *
         * 目标位置已经在 M29_RELEASING -> CSP 的切换入口保存，并由
         * Driver_Process_Mode_Switch 在 PREPARE_TARGET 阶段再次装载，
         * 因此 READY 阶段只保持该值，不再改写。
        */
        spindle_log_m29_csp_trace();
        Driver_Enable(&spindle_driver);
        return;
    }

    if (spl_context.csv_switch_pending)
    {
        if (spindle_driver.act_mode == DRV_MODE_CSP &&
            !Driver_Is_Mode_Switching(&spindle_driver))
        {
            if (Driver_Request_Mode_Switch(&spindle_driver, DRV_MODE_CSV) !=
                HAL_DRIVER_SUCCESS)
            {
                LOG(LOG_LEVEL_ERR,
                    "主轴CSP切换CSV请求失败，继续保持停止："
                    "status_word=0x%04X state=%d",
                    spindle_driver.status_word, spindle_driver.state);
            }
            return;
        }

        if (Driver_Is_Mode_Switch_Done(&spindle_driver) &&
            spindle_driver.act_mode == DRV_MODE_CSV)
        {
            spl_context.csv_switch_pending = false;
            if (spl_context.command_state == SPINDLE_STOP)
            {
                spl_context.spl_event = SPINDLE_EVENT_STOP;
            }
            else
            {
                spl_context.spl_event =
                    (spl_context.command_state == SPINDLE_CCW) ?
                    SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW;
            }
        }
        else if (Driver_Get_Mode_Switch_Result(&spindle_driver) !=
                 HAL_DRIVER_SUCCESS)
        {
            LOG(LOG_LEVEL_ERR,
                "主轴CSP切换CSV失败，保留重试：status_word=0x%04X "
                "state=%d mode_switch_state=%d",
                spindle_driver.status_word, spindle_driver.state,
                spindle_driver.mode_switch_state);
            if (Driver_Request_Mode_Switch(&spindle_driver,
                                           DRV_MODE_CSV) !=
                HAL_DRIVER_SUCCESS)
            {
                Driver_Disable(&spindle_driver);
            }
        }
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

/* 将速度规划器的结果写入 HAL；实际 PDO 发送由 EtherCAT 主循环完成。 */
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


    spindle_finish_stop_if_ready();
}

/* 汇总会强制主轴停止的外部原因。 */
static bool spindle_stop_is_required(const spindle_context_t *context)
{
    /* 汇总所有必须停止主轴的原因，供周期入口统一仲裁。 */
    bool stop_required = false;

    if (spindle_driver_has_fault())
    {
        return true;
    }

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

/*
 * 停止请求：普通 M05 在 CSV 中受控减速；CSP、急停和故障走安全路径。
 */
static void spindle_handle_stop(spindle_context_t *context)
{
    /*
     * 停止事件只撤销速度目标。普通 M5 保持驱动器可受控减速，急停/Fault
     * 则直接请求下使能；最终是否完成由 spindle_finish_stop_if_ready 判断。
     */
    context->start_request_pending = false;
    context->deferred_start_pending = false;
    context->command_state = SPINDLE_STOP;
    context->command_speed = 0.0;
    context->spl_state = SPINDLE_STOP;
    context->vel_planner.enable = false;
    context->vel_planner.target_vel = 0.0;

    if (spindle_driver.act_mode == DRV_MODE_CSP)
    {
        bool preserve_m29_request;

        /*
         * 复位/急停不能等待 CSP->CSV 的正常减速握手。CSP 下没有速度
         * 目标可供本模块执行受控减速，继续发起模式切换会把 target_pos
         * 锁在当前反馈位置并可能长期卡在切换状态，复位后主轴仍保持
         * 使能。此时立即清零目标并下使能，由驱动器安全状态机接管停车。
         */
        if (spindle_reset_or_estop_active())
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
            context->csv_switch_pending = false;
            context->command_state = SPINDLE_STOP;
            /* 复位必须取消尚未完成的 CSP/CSV 模式切换；否则
             * Driver_Cyclic_Process 会优先继续处理旧切换并在超时后返回 -1。 */
            spindle_driver.mode_switch_state = DRV_MODE_SWITCH_IDLE;
            spindle_driver.mode_switch_result = HAL_DRIVER_SUCCESS;
            spindle_driver.mode_switch_cycles = 0U;
            spindle_driver.target_pos = spindle_driver.actual_pos;
            Hal_Driver_Set_Target_Pos(&spindle_driver,
                                      spindle_driver.actual_pos);
            Driver_Disable(&spindle_driver);
            Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
            return;
        }

        if (Driver_Is_Mode_Switching(&spindle_driver))
        {
            return;
        }
        preserve_m29_request = context->m29_active &&
                               context->spl_event == SPINDLE_EVENT_ORIENT;
        if (!preserve_m29_request)
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
        }
        context->command_state = SPINDLE_STOP;
        if (Driver_Request_Mode_Switch(&spindle_driver, DRV_MODE_CSV) ==
            HAL_DRIVER_SUCCESS)
        {
            context->csv_switch_pending = true;
        }
        else
        {
            Driver_Disable(&spindle_driver);
        }
        return;
    }

    /*
     * 急停或驱动器 Fault 时不能请求重新使能：驱动器已经无法执行
     * 正常减速，释放急停后也必须保持下使能，防止旧目标速度恢复。
     * 普通 M5 则保持 CSV 模式并允许驱动器执行受控减速。
     */
    if (!spindle_reset_or_estop_active() && !spindle_driver_has_fault())
    {
        Driver_Enable(&spindle_driver);
    }
    else
    {
        Driver_Disable(&spindle_driver);
    }
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
}

/* 复位/急停锁存期间，禁止旧的启动请求自动恢复主轴。 */
static bool spindle_reset_or_estop_active(void)
{
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        return true;
    }
    if (spl_context.reset_requires_start)
    {
        return true;
    }
    return spindle_reset_is_requested();
}

/*
 * 启动请求：如果当前仍在 CSP，先请求 CSP -> CSV；否则直接接管 CSV。
 * 注意：这里只写 req_mode，不能伪造 act_mode，实际模式必须由驱动器确认。
 */
static void spindle_handle_rotation(spindle_context_t *context,
                                    spindle_state_t state)
{
    if (spindle_driver.act_mode == DRV_MODE_CSP &&
        !Driver_Is_Mode_Switching(&spindle_driver))
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
        context->command_state = state;
        context->command_speed = spindle_get_selected_speed(context);
        /* 新的 M3/M4 明确取得控制权，解除 M29 完成后的停止锁。 */
        context->orient_hold_stop = false;
        context->reset_requires_start = false;
        context->start_request_pending = false;
        if (Driver_Request_Mode_Switch(&spindle_driver, DRV_MODE_CSV) ==
            HAL_DRIVER_SUCCESS)
        {
            context->csv_switch_pending = true;
        }
        else
        {
            /* 模式切换失败不能丢失启动请求，后续周期继续重试。 */
            context->csv_switch_pending = true;
        }
        return;
    }

    if (context->m29_phase == SPINDLE_M29_READY)
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
    }

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
    if (context->reset_requires_start && !context->start_request_pending)
    {
        return;
    }
    context->spl_state = state;
    context->reset_requires_start = false;
    context->start_request_pending = false;
    context->mcode_feedback = false;
    context->spl_vel_state = SPINDLE_VEL_RAMP;
    context->orient_hold_stop = false;
    Driver_Request_Mode(&spindle_driver, DRV_MODE_CSV);
    Driver_Enable(&spindle_driver);

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

/* 计算最终有效目标：S 值经倍率、最大转速限制和方向修正后的结果。 */
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

/* 软件规划器到位后，再叠加 PDO 实际速度确认，防止提前生成 FIN。 */
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

/*
 * 统一的“到速反馈屏蔽”条件。只要主轴控制权还未稳定交接，F45.3
 * 就必须保持低电平，即使 spl_vel_state 中残留 ARRIVED 也不能发布。
 */
static bool spindle_velocity_feedback_blocked(void)
{
    if (spindle_stop_is_required(&spl_context) || spl_context.reset_requires_start)
    {
        return true;
    }
    /* 新请求尚未接管时，上一条指令的到速或零速都不能确认新指令。 */
    if (spl_context.start_request_pending || spl_context.deferred_start_pending)
    {
        return true;
    }
    if (spl_context.spl_event != SPINDLE_EVENT_NONE)
    {
        return true;
    }
    if (spl_context.orient_request_pending || spl_context.orient_release_pending)
    {
        return true;
    }
    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE || spl_context.orient_hold_stop)
    {
        return true;
    }
    if (spindle_driver.orient_state != DRV_ORIENT_IDLE)
    {
        return true;
    }
    if (spl_context.csv_switch_pending || Driver_Is_Mode_Switching(&spindle_driver))
    {
        return true;
    }
    if (spl_context.m29_active || spl_context.m29_phase != SPINDLE_M29_IDLE)
    {
        return true;
    }
    return spl_context.command_state != spl_context.spl_state;
}

/* 停止完成与旋转到速是两套独立判定，停止完成只确认零速。 */
static bool spindle_stop_is_complete(void)
{
    if (spindle_velocity_feedback_blocked())
    {
        return false;
    }
    if (spl_context.spl_state != SPINDLE_STOP)
    {
        return false;
    }
    if (fabs(spl_context.vel_planner.current_vel) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    if (fabs(spl_context.vel_planner.target_vel) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    return fabs((double)spindle_driver.actual_vel) <= SPINDLE_ACTUAL_ZERO_EPSILON;
}

/*
 * 旋转到速判定：有效目标、规划器、实际 PDO、CSV 和 Operation Enabled
 * 必须在同一个控制周期内全部成立。
 */
static bool spindle_is_velocity_arrived(void)
{
    if (spindle_velocity_feedback_blocked() || !spindle_is_rotating())
    {
        return false;
    }
    if (spindle_driver.act_mode != DRV_MODE_CSV)
    {
        return false;
    }
    if (spindle_driver.state != DRV_STATE_OPERATION_ENABLED)
    {
        return false;
    }
    if ((spindle_driver.status_word & 0x006FU) != 0x0027U)
    {
        return false;
    }
    if (!spl_context.vel_planner.enable)
    {
        return false;
    }

    const double target = spindle_calculate_target_velocity(&spl_context);
    const double actual = (double)spindle_driver.actual_vel;
    const double tolerance = fmax(SPINDLE_SPEED_ARRIVAL_MIN_TOL,
                                  fabs(target) * SPINDLE_SPEED_ARRIVAL_TOL);
    if (!isfinite(target) || fabs(target) <= SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    /* 误差带不能掩盖零速或反方向反馈，尤其是低速目标。 */
    if (fabs(actual) <= SPINDLE_ACTUAL_SPEED_EPSILON || actual * target <= 0.0)
    {
        return false;
    }
    /* 规划器必须已追上当前有效目标，不能只比较 actual 与斜坡中间速度。 */
    if (fabs(spl_context.vel_planner.target_vel - target) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    if (fabs(spl_context.vel_planner.current_vel - target) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    return fabs(actual - target) <= tolerance;
}

/* 将底层 Driver 定向阶段映射到 NC/PLC 可见的定向阶段。 */
static void spindle_update_orient_state(void)
{
    static bool release_status_log_active;
    static drv_state_t last_release_driver_state;
    static drv_state_t last_release_target_state;
    static uint16_t last_release_status_word;
    static drv_orient_state_t last_release_orient_state;

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
                spl_context.deferred_start_pending = false;
            }
            else
            {
                spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
                spl_context.orient_release_pending = true;
                spl_context.m19_active = false;
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

    /* 只有状态字确认 Ready to Switch On，才结束释放握手。 */
    if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING &&
        Driver_Is_Ready_to_Switch_On(&spindle_driver))
    {
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
        spl_context.orient_hold_stop = false;
        /* 驱动器在定向期间控制速度；交还 CSV 时仅同步一次起始速度。 */
        /* 禁止把此同步移到普通周期：每周期追随反馈会破坏加减速规划。 */
        spl_context.vel_planner.current_vel = (double)spindle_driver.actual_vel;
        spl_context.vel_planner.target_vel = 0.0;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.active = false;
        spindle_clear_velocity_arrival();
        spindle_apply_deferred_start();
        release_status_log_active = false;
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING)
    {
        bool release_status_changed;

        const bool driver_state_changed = last_release_driver_state != spindle_driver.state;
        const bool target_state_changed = last_release_target_state != spindle_driver.target_state;
        const bool status_word_changed = last_release_status_word != spindle_driver.status_word;
        const bool orient_state_changed = last_release_orient_state != spindle_driver.orient_state;
        release_status_changed = !release_status_log_active;
        release_status_changed |= driver_state_changed;
        release_status_changed |= target_state_changed;
        release_status_changed |= status_word_changed;
        release_status_changed |= orient_state_changed;

        if (release_status_changed)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴定向释放状态：state=%d target_state=%d "
                "status_word=0x%04X orient_state=%d actual_vel=%d",
                spindle_driver.state,
                spindle_driver.target_state,
                spindle_driver.status_word,
                spindle_driver.orient_state,
                spindle_driver.actual_vel);
            release_status_log_active = true;
            last_release_driver_state = spindle_driver.state;
            last_release_target_state = spindle_driver.target_state;
            last_release_status_word = spindle_driver.status_word;
            last_release_orient_state = spindle_driver.orient_state;
        }

        if (Driver_Is_Ready_to_Switch_On(&spindle_driver))
        {
            return;
        }

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
            spl_context.deferred_start_pending = false;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    if (spl_context.orient_phase == SPINDLE_ORIENT_DONE &&
        spl_context.m19_active)
    {
        spl_context.m19_active = false;
        spindle_report_mcode_complete();
    }
}

/* M29 的 NC 动作流程：定向 -> 释放 -> CSP 切换 -> READY。 */
static void spindle_update_m29(void)
{
    if (!spl_context.m29_active)
    {
        return;
    }

    /* 使用顺序 if 而不是 switch：某阶段完成后可在本周期继续推进下一阶段。
     * 各阶段入口仍重新读取当前阶段，不能把所有条件提前缓存。 */
    const bool orientation_lost = spl_context.orient_phase == SPINDLE_ORIENT_RELEASING ||
                                  spl_context.orient_phase == SPINDLE_ORIENT_IDLE;
    if (spl_context.m29_phase == SPINDLE_M29_ORIENTING && orientation_lost)
    {
        LOG(LOG_LEVEL_ERR,
            "M29主轴定向失败：orient_phase=%d status_word=0x%04X "
            "actual_vel=%d",
            spl_context.orient_phase, spindle_driver.status_word,
            spindle_driver.actual_vel);
        spl_context.m29_phase = SPINDLE_M29_ERROR;
        spl_context.reset_requires_start = true;
        spl_context.start_request_pending = false;
        spl_context.orient_hold_stop = true;
        spl_context.command_speed = 0.0;
        spl_context.command_state = SPINDLE_STOP;
        return;
    }

    if (spl_context.m29_phase == SPINDLE_M29_ORIENTING &&
        spl_context.orient_phase == SPINDLE_ORIENT_DONE)
    {
        if (spl_orient_release())
        {
            spl_context.m29_phase = SPINDLE_M29_RELEASING;
            spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
        }
        else
        {
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spl_context.orient_hold_stop = true;
        }
    }

    const bool drive_released = Driver_Is_Ready_to_Switch_On(&spindle_driver);
    const bool actual_stopped = fabs((double)spindle_driver.actual_vel) <=
                                SPINDLE_ACTUAL_ZERO_EPSILON;
    const bool ready_to_enter_csp = drive_released && actual_stopped;
    if (spl_context.m29_phase == SPINDLE_M29_RELEASING && ready_to_enter_csp)
    {
        spindle_driver.target_pos = spindle_driver.actual_pos;
        Hal_Driver_Set_Target_Pos(&spindle_driver, spindle_driver.actual_pos);
        LOG(LOG_LEVEL_INFO,
            "M29定向停止确认，切换CSP保持：actual_pos=%d actual_vel=%d "
            "status_word=0x%04X",
            spindle_driver.actual_pos, spindle_driver.actual_vel,
            spindle_driver.status_word);
        if (Driver_Request_Mode_Switch(&spindle_driver, DRV_MODE_CSP) ==
            HAL_DRIVER_SUCCESS)
        {
            spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
            spl_context.m29_phase = SPINDLE_M29_SWITCHING_CSP;
            spl_context.orient_elapsed_cycles = 0U;
        }
        else
        {
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spl_context.orient_hold_stop = true;
        }
    }

    if (spl_context.m29_phase == SPINDLE_M29_SWITCHING_CSP)
    {
        if (spindle_csp_switch_is_complete())
        {
            spl_context.m29_phase = SPINDLE_M29_READY;
        }
        else if (Driver_Get_Mode_Switch_Result(&spindle_driver) !=
                 HAL_DRIVER_SUCCESS)
        {
            LOG(LOG_LEVEL_ERR,
                "M29切换CSP失败：status_word=0x%04X state=%d "
                "target_state=%d mode_switch_state=%d",
                spindle_driver.status_word, spindle_driver.state,
                spindle_driver.target_state,
                spindle_driver.mode_switch_state);
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spl_context.start_request_pending = false;
            spl_context.orient_hold_stop = true;
            spl_context.command_speed = 0.0;
            spl_context.command_state = SPINDLE_STOP;
        }
        else if (++spl_context.orient_elapsed_cycles >=
                 SPINDLE_M29_MODE_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR,
                "M29切换CSP超时：status_word=0x%04X state=%d "
                "target_state=%d mode_switch_state=%d act_mode=%d",
                spindle_driver.status_word, spindle_driver.state,
                spindle_driver.target_state, spindle_driver.mode_switch_state,
                spindle_driver.act_mode);
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spl_context.start_request_pending = false;
            spl_context.orient_hold_stop = true;
            spl_context.command_speed = 0.0;
            spl_context.command_state = SPINDLE_STOP;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }
}

/* 停止完成后产生一次内部完成反馈，供 PLC F45.3 使用。 */
static void spindle_finish_stop_if_ready(void)
{
    if (!spindle_stop_is_complete())
    {
        return;
    }

    if (spindle_driver.act_mode != DRV_MODE_NONE)
    {
        Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
        spindle_disable_driver();
    }

    /* 停止完成单独确认；待启动、定向和模式切换均不能走到这里。 */
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spindle_report_mcode_complete();
}

/* 定向释放确认后，在同一周期接管延迟的 M03/M04 请求。 */
static void spindle_apply_deferred_start(void)
{
    if (spindle_stop_is_required(&spl_context) || spl_context.reset_requires_start)
    {
        spl_context.deferred_start_pending = false;
        spl_context.start_request_pending = false;
        return;
    }
    if (spl_context.m29_active)
    {
        return;
    }

    /* 后到的 CNC 命令优先于先前排队的 PLC 方向边沿。 */
    if (spl_context.deferred_start_pending && !spl_context.start_request_pending)
    {
        spl_context.command_state = spl_context.deferred_start_state;
        spl_context.command_speed = spl_context.deferred_start_speed;
        spl_context.start_request_pending = true;
    }
    spl_context.deferred_start_pending = false;
    if (!spl_context.start_request_pending)
    {
        return;
    }
    if (spl_context.command_state == SPINDLE_STOP)
    {
        return;
    }
    spindle_handle_rotation(&spl_context, spl_context.command_state);
}

static void spindle_disable_driver_before_operation(void)
{
    /* 驱动器尚未进入 Operation Enabled 时，不能做受控减速，只能清零并下使能。 */
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);
    spindle_disable_driver();
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
    /* 仅锁存 NC 内部完成状态，不写 G4.3；PLC FIN 始终由 PLC 提供。 */
    if (!spl_context.mcode_feedback)
    {
        spl_context.mcode_feedback = true;
    }
}

/* 只在首次采样或 CSP 目标变化时记录诊断，避免 1 ms 周期刷屏。 */
static void spindle_log_m29_csp_trace(void)
{
    static int32_t last_target_pos;
    static bool target_initialized;
    bool target_changed;

    target_changed = !target_initialized ||
                     last_target_pos != spindle_driver.target_pos;

    /* 首次进入或固定目标变化时记录；实际位置变化不单独触发此日志。
     * 控制权/握手变化时的实际位置偏差由 spindle_trace_status 补充记录。 */
    if (target_changed)
    {
        LOG(LOG_LEVEL_INFO,
            "M29 CSP跟踪：target_pos=%d target_vel=%d actual_pos=%d actual_vel=%d "
            "act_mode=%d req_mode=%d state=%d target_state=%d "
            "status_word=0x%04X control_word=0x%04X mode_switch=%d",
            spindle_driver.target_pos, spindle_driver.target_vel, spindle_driver.actual_pos,
            spindle_driver.actual_vel, spindle_driver.act_mode,
            spindle_driver.req_mode, spindle_driver.state,
            spindle_driver.target_state, spindle_driver.status_word,
            spindle_driver.control_word, spindle_driver.mode_switch_state);
    }

    last_target_pos = spindle_driver.target_pos;
    target_initialized = true;
}
